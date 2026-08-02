#!/usr/bin/env python3
"""MQTT adapter for the rover's existing ROS /goto_cell action."""
from __future__ import annotations
import json, math, os, queue, re, subprocess, sys, threading, time
from urllib.request import urlopen
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import CompressedImage, Image
from td_interfaces.action import GoToCell
from td_world.field import load_field
from td_world.grid import Transform2D

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "common"))
from mqtt import MqttClient

cfg = json.loads((Path(__file__).with_name("device.json")).read_text())
AGENT = cfg["agent_id"]
def topic(suffix): return f"air-watch/v1/{AGENT}/{suffix}"

MOTION_EXECUTOR = cfg.get("motion_executor_path",
    "/home/pi/sverk_rover/install/rover_web/share/rover_web/tools/rover_motion_executor.py")
MAX_LINEAR_SPEED = 0.35
MAX_LATERAL_SPEED = 0.35
MAX_ANGULAR_SPEED = 1.5

class RoverAgent(Node):
 def __init__(self):
  super().__init__("air_watch_rover")
  self.commands, self.current_cell, self.phase, self.active_goal = queue.Queue(), None, "idle", None
  self.goal_epoch=0
  self.goto_sent_at=0.0
  self.last_feedback_key, self.last_feedback_at, self.last_amcl = None, 0.0, 0.0
  self.map_pose = None
  self.web_pose_at=0.0; self.position_source="waiting for rover web API"
  self.web_api_url=cfg.get("web_api_url","http://127.0.0.1:8767/v1/state")
  self.web_calibration_url=cfg.get("web_calibration_url","http://127.0.0.1:8767/v1/field-calibration")
  self.field = load_field(cfg.get("field_config", "/home/pi/install/td_bringup/share/td_bringup/config/field.yaml"))
  self.goto = ActionClient(self, GoToCell, "/goto_cell")
  self.goto_process=None
  # Manual drive mirrors the rover_control_api teleop: a latched Twist on
  # /cmd_vel_teleop (twist_mux priority 100) that expires after ttl_ms.
  self.teleop_publisher=self.create_publisher(Twist,cfg.get("teleop_topic","/cmd_vel_teleop"),10)
  self.drive_cmd=None; self.drive_expires=0.0; self.drive_active=False
  self.stop_latched=False
  self.last_command_key=None; self.last_command_at=0.0
  self.motion_process=None
  self.initial_pose_publisher=self.create_publisher(PoseWithCovarianceStamped,cfg.get("initial_pose_topic","/initialpose"),10)
  # The latest ROS camera image is held until the dashboard requests OCR.
  # RapidOCR is imported lazily so missing OCR packages never stop navigation.
  self.camera_topic=cfg.get("camera_topic", "/camera/image_raw")
  self.latest_image=None; self.image_sequence=0; self.image_ready=threading.Event()
  self.ocr_busy=threading.Lock()
  image_type=CompressedImage if self.camera_topic.endswith("/compressed") else Image
  self.create_subscription(image_type, self.camera_topic, self._image,
                           QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
  # AMCL keeps its last pose transient-local.  Request that latched sample so
  # the dashboard has a cell immediately after agent restart, even while parked.
  self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl,
                           QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.mqtt = MqttClient(cfg["mqtt_host"], cfg["mqtt_port"], client_id="air-watch-rover", on_message=self._mqtt_message, logger=lambda x:self.get_logger().info(x))
  self.mqtt.start()
  if not self.mqtt.wait_connected(10): raise RuntimeError("MQTT broker is unavailable")
  self.mqtt.subscribe(topic("cmd"), qos=1)
  self._ensure_goto_server()
  self.create_timer(.05, self._process_commands); self.create_timer(.05, self._drive_timer); self.create_timer(1.0, self._status); self.create_timer(1.0, self._web_pose); self.create_timer(1.0, self._goto_watchdog)
  self.event("ready", text=f"rover MQTT adapter ready; camera OCR listens to {self.camera_topic}")
 def event(self, name, **data): self.mqtt.publish(topic("evt"), json.dumps({"agent":AGENT,"name":name,"data":data}), qos=1)
 def _amcl(self, message):
  import time
  # The rover web API owns the current localization view.  AMCL is only a
  # fallback while that local API has not delivered a recent pose.
  if time.monotonic()-self.web_pose_at < 2.5: return
  self.last_amcl=time.monotonic()
  position=message.pose.pose.position
  self._set_map_pose(float(position.x),float(position.y),"AMCL fallback")
 def _set_map_pose(self, x, y, source):
  self.map_pose=(x,y); self.position_source=source
  cell=self.field.cell_from_map(x,y)
  name=cell.name if cell is not None else None
  if name!=self.current_cell:
   self.current_cell=name
   self.event("log",text=f"{source}: map position {x:.2f}, {y:.2f} -> field cell {name or 'outside configured field'}")
 def _web_pose(self):
  """Use the same AMCL/map pose which the rover's own web control exposes."""
  try:
   with urlopen(self.web_api_url,timeout=.35) as response: data=json.loads(response.read().decode())
   pose=data.get("pose") or {}
   if pose.get("source")!="amcl" or pose.get("frame_id")!="map": return
   self._web_calibration()
   x,y=float(pose["x"]),float(pose["y"])
   if not all(map(math.isfinite,(x,y))): return
   self.web_pose_at=time.monotonic(); self._set_map_pose(x,y,"rover web API")
  except Exception: pass
 def _web_calibration(self):
  """Match the cell transform to corners stored in rover_control_api."""
  try:
   with urlopen(self.web_calibration_url,timeout=.35) as response: data=json.loads(response.read().decode())
   calibration=data.get("calibration") if data.get("configured") else None
   corners=calibration.get("corners") if calibration else None
   if not isinstance(corners,dict): return
   width,height=self.field.grid.width,self.field.grid.height
   source=[(0.0,0.0),(width,0.0),(width,height),(0.0,height)]
   destination=[(float(corners[n]["x"]),float(corners[n]["y"])) for n in ("bottom_left","bottom_right","top_right","top_left")]
   self.field.field_to_map=Transform2D.fit(source,destination)
  except Exception: pass
 def _calibrated_cell(self, column, row, yaw_field_deg=0.0):
  """Same cell -> map pose the web's initial-cell/goal-cell use (bilinear corners)."""
  with urlopen(self.web_calibration_url,timeout=.5) as response: data=json.loads(response.read().decode())
  calibration=data.get("calibration") if data.get("configured") else None
  corners=calibration.get("corners") if calibration else None
  if not isinstance(corners,dict): raise RuntimeError("field grid not configured in the web control API")
  columns=int(calibration.get("columns",6)); rows=int(calibration.get("rows",6))
  if not (1<=column<=columns and 1<=row<=rows):
   raise ValueError(f"cell must be within 1..{columns} x 1..{rows}")
  u=(column-0.5)/columns; v=(row-0.5)/rows
  bl,br=corners["bottom_left"],corners["bottom_right"]
  tr,tl=corners["top_right"],corners["top_left"]
  x=((1-u)*(1-v)*float(bl["x"])+u*(1-v)*float(br["x"])+u*v*float(tr["x"])+(1-u)*v*float(tl["x"]))
  y=((1-u)*(1-v)*float(bl["y"])+u*(1-v)*float(br["y"])+u*v*float(tr["y"])+(1-u)*v*float(tl["y"]))
  dx=(1-u)*(float(tl["x"])-float(bl["x"]))+u*(float(tr["x"])-float(br["x"]))
  dy=(1-u)*(float(tl["y"])-float(bl["y"]))+u*(float(tr["y"])-float(br["y"]))
  field_y_axis_yaw_deg=math.degrees(math.atan2(dy,dx))
  return {"map_label":str(calibration.get("map_label","")),"x":x,"y":y,"yaw_deg":field_y_axis_yaw_deg-yaw_field_deg}
 def set_initial_pose(self,args):
  """Publish /initialpose at a field cell (default (1,3)=D1) or an explicit point.

  yaw_deg is the map-frame orientation of the published pose (like a manual
  /initialpose), not a field-relative offset.  Default 95 degrees.
  """
  try:
   cell_text=str(args.get("cell","")).strip()
   yaw_deg=float(args.get("yaw_deg",95.0))
   if cell_text:
    if "," in cell_text or re.search(r"[\s:;]",cell_text):
     parts=re.split(r"[\s,:;]+",cell_text.strip())
     column=int(parts[0]); row=int(parts[1])
    else:
     ref=self.field.cell(cell_text.upper())
     column=ref.col+1; row=self.field.grid.rows-ref.row
   else:
    column=int(args.get("column",1)); row=int(args.get("row",3))
   x,y=args.get("x"),args.get("y")
   if x is not None and y is not None:
    x,y=float(x),float(y)
    label=f"point ({x:.2f},{y:.2f})"
   else:
    pose=self._calibrated_cell(column,row,0.0)
    x,y=pose["x"],pose["y"]
    label=f"cell ({column},{row})"
   self._publish_initial_pose(x,y,yaw_deg)
   self.map_pose=(x,y); self.web_pose_at=time.monotonic()
   cell=self.field.cell_from_map(x,y)
   if cell is not None: self.current_cell=cell.name
   self.event("log",text=f"initial pose {label} -> map x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f} deg")
   self.event("result",command="set_initial_pose",success=True,text=f"initial pose set at {label} (map x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f})")
  except Exception as exc:
   self.event("result",command="set_initial_pose",success=False,text=f"set_initial_pose failed: {exc}")
 def _publish_initial_pose(self,x,y,yaw_deg):
  yaw=math.radians(yaw_deg)
  message=PoseWithCovarianceStamped()
  message.header.frame_id="map"; message.header.stamp=self.get_clock().now().to_msg()
  message.pose.pose.position.x=float(x); message.pose.pose.position.y=float(y)
  message.pose.pose.orientation.z=math.sin(yaw/2.0); message.pose.pose.orientation.w=math.cos(yaw/2.0)
  message.pose.covariance[0]=0.25; message.pose.covariance[7]=0.25; message.pose.covariance[35]=0.0685
  self.initial_pose_publisher.publish(message)
 def _drive_timer(self):
  """Hold the rover still after a stop; keep the latched teleop Twist fresh."""
  now=time.monotonic()
  if self.stop_latched:
   self.teleop_publisher.publish(Twist()); return
  if self.drive_active and now<self.drive_expires:
   self.teleop_publisher.publish(self.drive_cmd)
  elif self.drive_active:
   self.drive_active=False; self.teleop_publisher.publish(Twist())
 def drive(self,args):
  """Manual drive: publish Twist on /cmd_vel_teleop for ttl_ms (mirrors /v1/teleop)."""
  self.stop_latched=False
  try:
   linear_x=float(args.get("linear_x",0.0)); linear_y=float(args.get("linear_y",0.0))
   angular_z=float(args.get("angular_z",0.0))
   ttl_ms=float(args.get("ttl_ms",args.get("duration_ms",1000)))
   if not all(map(math.isfinite,(linear_x,linear_y,angular_z))):
    self.event("result",command="drive",success=False,text="non-finite drive values"); return
   if self.active_goal is not None:
    self.active_goal.cancel_goal_async(); self.active_goal=None; self.phase="idle"
   command=Twist()
   command.linear.x=max(-MAX_LINEAR_SPEED,min(MAX_LINEAR_SPEED,linear_x))
   command.linear.y=max(-MAX_LATERAL_SPEED,min(MAX_LATERAL_SPEED,linear_y))
   command.angular.z=max(-MAX_ANGULAR_SPEED,min(MAX_ANGULAR_SPEED,angular_z))
   self.drive_cmd=command; self.drive_expires=time.monotonic()+max(0.1,min(ttl_ms,30000.0))/1000.0; self.drive_active=True
   self.teleop_publisher.publish(command)
   self.event("log",text=f"drive lx={linear_x:.2f} ly={linear_y:.2f} az={angular_z:.2f} for {ttl_ms:.0f} ms")
   self.event("result",command="drive",success=True,text=f"driving lx={command.linear.x:.2f} ly={command.linear.y:.2f} az={command.angular.z:.2f} for {ttl_ms:.0f} ms")
  except Exception as exc:
   self.event("result",command="drive",success=False,text=f"drive failed: {exc}")
 def stop_drive(self):
  self.drive_active=False; self.drive_cmd=None; self.teleop_publisher.publish(Twist())
 def run_motion(self,kind,args,command):
  """Closed-loop move/turn via the same rover_motion_executor.py the web uses."""
  self.stop_latched=False
  if self.motion_process is not None and self.motion_process.poll() is None:
   self.event("result",command=command,success=False,text="a motion command is already running; send stop first"); return
  if self.active_goal is not None:
   self.event("result",command=command,success=False,text="rover is navigating; send stop first"); return
  if not Path(MOTION_EXECUTOR).is_file():
   self.event("result",command=command,success=False,text=f"motion executor not found: {MOTION_EXECUTOR}"); return
  if kind=="move":
   forward=float(args.get("forward",0.0)); left=float(args.get("left",0.0)); speed=float(args.get("speed",0.18))
   if abs(forward)>5.0 or abs(left)>5.0:
    self.event("result",command=command,success=False,text="move distance is limited to 5 m per command"); return
   speed=max(0.10,min(speed,0.35))
   extra=["move","--forward",str(forward),"--left",str(left),"--speed",str(speed)]
  else:
   degrees=float(args.get("degrees",0.0)); speed=float(args.get("speed",0.32)); tolerance=float(args.get("tolerance_deg",3.0))
   if abs(degrees)>720.0:
    self.event("result",command=command,success=False,text="turn is limited to 720 degrees per command"); return
   speed=max(0.10,min(speed,1.0)); tolerance=max(1.0,min(tolerance,15.0))
   extra=["turn",str(degrees),"--speed",str(speed),"--tolerance-deg",str(tolerance)]
  command_line=("source /opt/ros/jazzy/setup.bash; "
                "source /home/pi/sverk_rover/install/setup.bash 2>/dev/null || true; "
                "source /home/pi/install/setup.bash; "
                f"exec python3 {MOTION_EXECUTOR} --odom-topic /odom --cmd-vel-topic /cmd_vel --drive-type differential "+" ".join(extra))
  environment=os.environ.copy(); environment["ROS_DOMAIN_ID"]=str(cfg.get("ros_domain_id",77))
  log=open("/home/pi/air-watch-motion.log","a",buffering=1)
  self.motion_process=subprocess.Popen(["bash","-lc",command_line],stdout=log,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
  self.phase=f"{kind} running"
  self.event("log",text=f"{command}: motion executor started (pid {self.motion_process.pid})")
  threading.Thread(target=self._motion_worker,args=(command,),daemon=True).start()
 def _motion_worker(self,command):
  process=self.motion_process
  code=process.wait()
  self.motion_process=None; self.phase="idle"
  ok=code==0
  self.event("log",text=f"{command}: motion executor exited with code {code}")
  self.event("result",command=command,success=ok,text=f"{command} finished (exit {code})")
 def _image(self, message):
  self.latest_image=message; self.image_sequence+=1; self.image_ready.set()
 def _ensure_goto_server(self):
  """Bring up the missing TD-to-Nav2 bridge with the rover's ROS domain."""
  if self.goto.wait_for_server(timeout_sec=2.0): return True
  if self.goto_process is None or self.goto_process.poll() is not None:
   # Duplicate servers cause Nav2 goal preemption; keep exactly one.
   subprocess.run(["pkill","-f","goto_cell"],capture_output=True)
   field_path=cfg.get("field_config", "/home/pi/install/td_bringup/share/td_bringup/config/field.yaml")
   command=("source /opt/ros/jazzy/setup.bash; "
    "source /home/pi/sverk_rover/install/setup.bash 2>/dev/null || true; "
    "source /home/pi/install/setup.bash; "
    f"exec ros2 run td_rover_executor goto_cell --ros-args -p field_config:={field_path}")
   environment=os.environ.copy(); environment["ROS_DOMAIN_ID"]=str(cfg.get("ros_domain_id",77))
   log=open("/home/pi/air-watch-goto.log","a",buffering=1)
   self.goto_process=subprocess.Popen(["bash","-lc",command],stdout=log,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
   self.event("log",text="/goto_cell was absent; starting TD-to-Nav2 bridge")
  return self.goto.wait_for_server(timeout_sec=4.0)
 def _mqtt_message(self, _topic, raw):
  try:
   command=json.loads(raw.decode() if isinstance(raw,bytes) else raw)
   self.get_logger().info(f"MQTT command received: {command.get('name')} {command.get('args',{})}")
   self.commands.put_nowait(command)
  except Exception as exc: self.get_logger().warn(f"invalid MQTT command: {exc}")
 def _process_commands(self):
  while not self.commands.empty():
   command=self.commands.get_nowait(); name,args=command.get("name"),command.get("args",{})
   cell=str(args.get("cell","")); key=(name,cell); now=time.monotonic()
   if key==self.last_command_key and now-self.last_command_at<5.0:
    self.event("log",text=f"duplicate command {name} {cell} ignored")
    continue
   self.last_command_key,self.last_command_at=key,now
   self.event("accepted",text=name,command=name); self.event("log",text=f"received MQTT command {name}: {args}")
   if name=="goto": self.goto_cell(str(args.get("cell", "")))
   elif name=="set_initial_pose": self.set_initial_pose(args)
   elif name in ("drive","teleop"): self.drive(args)
   elif name=="move": self.run_motion("move",args,name)
   elif name=="turn": self.run_motion("turn",args,name)
   elif name in ("read_text", "ocr"): self.read_text(float(args.get("timeout",8.0)))
   elif name in ("stop","emergency_stop"): self.cancel_motion(name)
   else: self.event("result",command=name,success=False,text=f"unsupported command {name}")
 def read_text(self, timeout):
  """Wait for a fresh ROS camera frame and recognise it outside the ROS loop."""
  if not self.ocr_busy.acquire(blocking=False):
   self.event("result",command="read_text",success=False,text="OCR is already processing a camera frame"); return
  timeout=max(1.0,min(timeout,20.0)); sequence=self.image_sequence; self.image_ready.clear()
  self.event("log",text=f"OCR: waiting for a fresh frame on {self.camera_topic} (up to {timeout:.0f} s)")
  threading.Thread(target=self._ocr_worker,args=(sequence,timeout),name="rover-ocr",daemon=True).start()
 def _ocr_worker(self, sequence, timeout):
  try:
   deadline=time.monotonic()+timeout
   while self.image_sequence<=sequence and time.monotonic()<deadline:
    self.image_ready.wait(timeout=min(.25,deadline-time.monotonic()))
   message=self.latest_image
   if message is None or self.image_sequence<=sequence:
    text=f"OCR: no camera frame received on {self.camera_topic} within {timeout:.0f} s"
    self.event("ocr",text=text,ok=False); self.event("result",command="read_text",success=False,text=text); return
   image=self._image_to_cv(message)
   try: from rapidocr_onnxruntime import RapidOCR
   except ImportError:
    text="OCR unavailable: install rapidocr_onnxruntime on the rover"
    self.event("ocr",text=text,ok=False); self.event("result",command="read_text",success=False,text=text); return
   self.event("log",text="OCR: recognising text with RapidOCR")
   result,_elapsed=RapidOCR()(image)
   text="\n".join(str(line[1]) for line in result) if result else ""
   shown=text or "No text found"
   self.event("ocr",text=shown,ok=True); self.event("result",command="read_text",success=True,text=shown)
  except Exception as exc:
   text=f"OCR failed: {exc}"
   self.event("ocr",text=text,ok=False); self.event("result",command="read_text",success=False,text=text)
  finally: self.ocr_busy.release()
 def _image_to_cv(self, message):
  """Convert usual ROS encodings without requiring cv_bridge."""
  import cv2, numpy as np
  if isinstance(message, CompressedImage):
   image=cv2.imdecode(np.frombuffer(message.data,dtype=np.uint8),cv2.IMREAD_COLOR)
   if image is None: raise ValueError("cannot decode compressed camera frame")
   return cv2.rotate(image, cv2.ROTATE_180)
  encoding=message.encoding.lower(); channels={"bgr8":3,"rgb8":3,"bgra8":4,"rgba8":4,"mono8":1}.get(encoding)
  if channels is None: raise ValueError(f"unsupported camera encoding {message.encoding!r}")
  row=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)[:, :message.width*channels]
  image=row.reshape(message.height,message.width,channels) if channels>1 else row.reshape(message.height,message.width)
  if encoding=="rgb8": return cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
  if encoding=="rgba8": return cv2.cvtColor(image,cv2.COLOR_RGBA2BGR)
  if encoding=="bgra8": return cv2.cvtColor(image,cv2.COLOR_BGRA2BGR)
  return image
 def goto_cell(self, cell):
  cell=cell.strip().upper()
  if not cell: self.event("result",command="goto",success=False,text="cell is empty"); return
  self.stop_latched=False
  if self.active_goal is not None: self.event("result",command="goto",success=False,text="rover is already moving; send stop first"); return
  self.event("log",text=f"goto {cell}: checking /goto_cell action server")
  if not self._ensure_goto_server():
   self.event("log",text=f"goto {cell}: /goto_cell is unavailable")
   self.event("result",command="goto",success=False,text="/goto_cell unavailable; start rover navigation stack")
   return
  import time
  if self.map_pose is None:
   self.event("log",text=f"goto {cell}: Nav2 has not published /amcl_pose yet (map -> base_link is unavailable)")
   self.event("result",command="goto",success=False,text="rover is not localized; set the initial pose on the rover map first")
   return
  try: ref=self.field.cell(cell)
  except Exception as exc: self.event("result",command="goto",success=False,text=f"invalid cell {cell}: {exc}"); return
  goal=GoToCell.Goal(); goal.goal.name=cell; goal.goal.row=ref.row; goal.goal.col=ref.col; goal.face_wall=False; goal.yaw_deg=float("nan"); goal.timeout=90.0
  self.phase=f"sending {cell}"; self.event("log",text=f"goto {cell}: sending goal to /goto_cell (timeout 90 s)")
  self.goto_sent_at=time.monotonic()
  self.goto.send_goal_async(goal,feedback_callback=self._feedback).add_done_callback(lambda f,destination=cell:self._goal_response(f,destination))
 def _goal_response(self,future,destination):
  try:
   handle=future.result()
   if handle is None or not handle.accepted: self.phase="idle"; self.event("log",text=f"goto {destination}: action server rejected goal"); self.event("result",command="goto",success=False,text=f"goto {destination} rejected by rover"); return
   self.active_goal=handle; self.phase=f"going to {destination}"; self.event("log",text=f"goto {destination}: accepted, waiting for Nav2")
   self.goal_epoch += 1
   epoch=self.goal_epoch
   handle.get_result_async().add_done_callback(lambda f,cell=destination,gen=epoch:self._goal_result(f,cell,gen))
  except Exception as exc: self.phase="idle"; self.event("log",text=f"goto {destination}: action error {exc}"); self.event("result",command="goto",success=False,text=f"goto {destination} failed: {exc}")
 def _feedback(self,feedback):
  info=feedback.feedback; cell=getattr(info.current_cell,"name","") or self.current_cell
  if cell: self.current_cell=cell
  self.phase=str(getattr(info,"phase","MOVING")); distance=float(getattr(info,"distance_remaining",0.0) or 0.0)
  key=(self.phase,cell)
  import time
  if key!=self.last_feedback_key or time.monotonic()-self.last_feedback_at>=2.0:
   self.last_feedback_key,self.last_feedback_at=key,time.monotonic()
   self.event("log",text=f"goto: phase={self.phase}, cell={cell or '-'}, remaining={distance:.2f} m")
 def _goal_result(self,future,destination,epoch=None):
  if epoch is not None and epoch!=self.goal_epoch: self.event("log",text=f"goto {destination}: stale result from superseded goal, ignored"); return
  self.active_goal=None; self.phase="idle"
  try:
   result=future.result().result; ok=bool(result.success)
   if ok: self.current_cell=result.final_cell.name or destination
   text=result.message or (f"arrived at {destination}" if ok else f"failed to reach {destination}")
   self.event("log",text=f"goto {destination}: completed: {text}")
   self.event("result",command="goto",success=ok,text=text)
  except Exception as exc: self.event("log",text=f"goto {destination}: result error {exc}"); self.event("result",command="goto",success=False,text=f"goto {destination} failed: {exc}")
 def cancel_motion(self,command):
  """Stop everything and hold the rover still until the next motion command."""
  stopped=False
  if self.active_goal is not None:
   self.goal_epoch += 1
   self.active_goal.cancel_goal_async(); self.phase="stopping"; self.event("log",text="goto: cancellation sent to /goto_cell"); stopped=True
   self.active_goal=None
  if self.drive_active:
   self.stop_drive(); self.event("log",text="drive: zero Twist published"); stopped=True
  if self.motion_process is not None and self.motion_process.poll() is None:
   try: self.motion_process.terminate()
   except Exception: pass
   self.event("log",text="motion executor: terminate requested"); stopped=True
  self.stop_latched=True
  self.teleop_publisher.publish(Twist())
  self.event("log",text="stop: zero Twist latched on /cmd_vel_teleop (rover held in place)")
  self.event("result",command=command,success=True,text="rover stopped and latched: "+("cancelled" if stopped else "no active motion"))
 def _goto_watchdog(self):
  """Surface goals that never reach /goto_cell (client wait-set hang)."""
  if self.phase.startswith("sending") and self.active_goal is None and time.monotonic()-self.goto_sent_at>10.0:
   self.phase="idle"
   self.teleop_publisher.publish(Twist())
 def _status(self):
  import time
  self.event("status",cell=self.current_cell,phase=self.phase,goto_ready=self.goto.server_is_ready(),nav_localized=self.map_pose is not None,map=list(self.map_pose) if self.map_pose else None,position_source=self.position_source,drive_active=self.drive_active,motion_running=self.motion_process is not None and self.motion_process.poll() is None,stop_latched=self.stop_latched)

def main():
 rclpy.init(); node=RoverAgent()
 try: rclpy.spin(node)
 except (KeyboardInterrupt, ExternalShutdownException): pass
 finally: node.mqtt.stop(); node.destroy_node(); rclpy.try_shutdown()
if __name__=="__main__": main()


