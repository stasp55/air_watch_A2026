#!/usr/bin/env python3
"""MQTT adapter for the rover's existing ROS /goto_cell action."""
from __future__ import annotations
import json, os, queue, subprocess, sys, threading, time
from urllib.request import urlopen
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
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

class RoverAgent(Node):
 def __init__(self):
  super().__init__("air_watch_rover")
  self.commands, self.current_cell, self.phase, self.active_goal = queue.Queue(), None, "idle", None
  self.last_feedback_key, self.last_feedback_at, self.last_amcl = None, 0.0, 0.0
  self.map_pose = None
  self.web_pose_at=0.0; self.position_source="waiting for rover web API"
  self.web_api_url=cfg.get("web_api_url","http://127.0.0.1:8767/v1/state")
  self.web_calibration_url=cfg.get("web_calibration_url","http://127.0.0.1:8767/v1/field-calibration")
  self.field = load_field(cfg.get("field_config", "/home/pi/install/td_bringup/share/td_bringup/config/field.yaml"))
  self.goto = ActionClient(self, GoToCell, "/goto_cell")
  self.goto_process=None
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
  self.create_timer(.05, self._process_commands); self.create_timer(1.0, self._status); self.create_timer(1.0, self._web_pose)
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
   if not all(map(__import__('math').isfinite,(x,y))): return
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
 def _image(self, message):
  self.latest_image=message; self.image_sequence+=1; self.image_ready.set()
 def _ensure_goto_server(self):
  """Bring up the missing TD-to-Nav2 bridge with the rover's ROS domain."""
  if self.goto.wait_for_server(timeout_sec=2.0): return True
  if self.goto_process is None or self.goto_process.poll() is not None:
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
   command=self.commands.get_nowait(); name,args=command.get("name",""),command.get("args",{})
   self.event("accepted",text=name,command=name); self.event("log",text=f"received MQTT command {name}: {args}")
   if name=="goto": self.goto_cell(str(args.get("cell", "")))
   elif name in ("read_text", "ocr"): self.read_text(float(args.get("timeout", 8.0)))
   elif name in ("stop","emergency_stop"): self.cancel_motion(name)
   else: self.event("result",command=name,success=False,text=f"unsupported rover command: {name}")
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
  goal=GoToCell.Goal(); goal.goal.name=cell; goal.face_wall=False; goal.yaw_deg=float("nan"); goal.timeout=90.0
  self.phase=f"sending {cell}"; self.event("log",text=f"goto {cell}: sending goal to /goto_cell (timeout 90 s)")
  self.goto.send_goal_async(goal,feedback_callback=self._feedback).add_done_callback(lambda f,destination=cell:self._goal_response(f,destination))
 def _goal_response(self,future,destination):
  try:
   handle=future.result()
   if handle is None or not handle.accepted: self.phase="idle"; self.event("log",text=f"goto {destination}: action server rejected goal"); self.event("result",command="goto",success=False,text=f"goto {destination} rejected by rover"); return
   self.active_goal=handle; self.phase=f"going to {destination}"; self.event("log",text=f"goto {destination}: accepted, waiting for Nav2")
   handle.get_result_async().add_done_callback(lambda f,cell=destination:self._goal_result(f,cell))
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
 def _goal_result(self,future,destination):
  self.active_goal=None; self.phase="idle"
  try:
   result=future.result().result; ok=bool(result.success)
   if ok: self.current_cell=result.final_cell.name or destination
   text=result.message or (f"arrived at {destination}" if ok else f"failed to reach {destination}")
   self.event("log",text=f"goto {destination}: completed: {text}")
   self.event("result",command="goto",success=ok,text=text)
  except Exception as exc: self.event("log",text=f"goto {destination}: result error {exc}"); self.event("result",command="goto",success=False,text=f"goto {destination} failed: {exc}")
 def cancel_motion(self,command):
  if self.active_goal is None: self.event("result",command=command,success=True,text="rover has no active route"); return
  self.active_goal.cancel_goal_async(); self.phase="stopping"; self.event("log",text="goto: cancellation sent to Nav2"); self.event("result",command=command,success=True,text="rover route cancellation requested")
 def _status(self):
  import time
  self.event("status",cell=self.current_cell,phase=self.phase,goto_ready=self.goto.server_is_ready(),nav_localized=self.map_pose is not None,map=list(self.map_pose) if self.map_pose else None,position_source=self.position_source)

def main():
 rclpy.init(); node=RoverAgent()
 try: rclpy.spin(node)
 except (KeyboardInterrupt, ExternalShutdownException): pass
 finally: node.mqtt.stop(); node.destroy_node(); rclpy.try_shutdown()
if __name__=="__main__": main()
