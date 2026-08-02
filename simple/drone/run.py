#!/usr/bin/env python3
"""Drone agent: one process for MQTT, camera/OpenCV and sverk flight API."""
from __future__ import annotations
import json, math, os, sys, queue, time, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]; sys.path[:0]=[str(ROOT/'common'),str(Path(__file__).parent)]
# ROS Humble cv_bridge needs the system NumPy 1.x even if NumPy 2 is installed.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from mqtt import MqttClient
from detector import detect

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import sverk_interfaces

# The VLM is an optional on-board ROS package.  Keeping this import optional
# means that the flight/vision agent remains usable while the model is not
# installed yet (or while its container is restarting).
try:
 from vlm_interfaces.action import VlmQuery
except ImportError:
 VlmQuery = None
try:
 from led_interfaces.srv import SetLEDEffect
except ImportError:
 SetLEDEffect = None
try:
 from aruco_det_loc.msg import MarkerArray
except ImportError:
 MarkerArray = None
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

cfg=json.loads((Path(__file__).with_name('device.json')).read_text())
field=json.loads((ROOT/'field.json').read_text())
GRID=field['grid']; agent=cfg['agent_id']
def topic(s): return f'air-watch/v1/{agent}/{s}'
def cell_to_map(name):
 row=ord(name[0].upper())-65; col=int(name[1:])-1; size=GRID['cell_size_m']
 x,y=(col+.5)*size,(GRID['rows']-row-.5)*size; t=field['field_to_map']; c,s=math.cos(t['yaw']),math.sin(t['yaw'])
 return t['x']+c*x-s*y,t['y']+s*x+c*y
def map_to_cell(x,y):
 t=field['field_to_map']; dx,dy=x-t['x'],y-t['y']; c,s=math.cos(t['yaw']),math.sin(t['yaw'])
 fx,fy=c*dx+s*dy,-s*dx+c*dy; col,row=int(fx/GRID['cell_size_m']),int(fy/GRID['cell_size_m'])
 if not(0<=col<GRID['cols'] and 0<=row<GRID['rows']): return None
 return f'{chr(65+GRID["rows"]-1-row)}{col+1}'
def map_to_field(x,y):
 t=field['field_to_map']; dx,dy=x-t['x'],y-t['y']; c,s=math.cos(t['yaw']),math.sin(t['yaw'])
 return [round(c*dx+s*dy,2),round(-s*dx+c*dy,2)]
ENEMY_IDS=set(int(v) for v in field.get('markers',{}).get('enemy_ids',[]) or [])
def corner_area(marker) -> float:
 corners=getattr(marker,'corners',None)
 if not corners or len(corners)<3: return 0.0
 total=0.0
 for i in range(len(corners)):
  x1,y1=corners[i].x,corners[i].y
  x2,y2=corners[(i+1) % len(corners)].x,corners[(i+1) % len(corners)].y
  total+=x1*y2-x2*y1
 return abs(total)/2.0

class Drone(Node):
 def __init__(self):
  # This node and the Sverk flight-API node must have different ROS names.
  super().__init__('air_watch_agent'); self.bridge= CvBridge(); self.image=None; self.info=None
  self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
  self.create_subscription(Image,cfg['camera_topic'],self.on_image,10)
  self.create_subscription(CameraInfo,cfg['camera_info_topic'],self.on_info,10)
  self.aruco_pose=None; self.aruco_time=0.0
  self.create_subscription(PoseWithCovarianceStamped,'/aruco/loc/pose_cov',self.on_aruco,10)
  self.enemy_tracking=False; self.enemy_emit_at=0.0
  if MarkerArray is not None:
   q=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST,depth=1)
   self.create_subscription(MarkerArray,cfg.get('enemy_markers_topic','/aruco/det/markers'),self.on_enemy_markers,q)
  self.commands=queue.Queue()
  self.debug_enabled=False; self.create_timer(0.33,self.debug_loop)
  self.api=sverk_interfaces.init(Nodename='air_watch_flight_api'); self.mqtt=MqttClient(cfg['mqtt_host'],cfg['mqtt_port'],client_id=f'air-watch-drone-{os.getpid()}',on_message=self.on_mqtt,logger=self.get_logger().info)
  # One permanent client is important: constructing/destroying ROS action
  # clients from callbacks corrupts the Humble executor wait-set on this stack.
  self.vlm_client=ActionClient(self,VlmQuery,cfg.get('vlm_action','/vlm/query')) if VlmQuery else None
  self.vlm_busy=False; self.vlm_last_request=0.0
  self.vlm_process=None
  self.led_client=self.create_client(SetLEDEffect,cfg.get('led_effect_service','/led_control/set_effect')) if SetLEDEffect else None
  self._blue_timer=None
  self.mqtt.start(); self.mqtt.wait_connected(10); self.mqtt.subscribe(topic('cmd'),qos=1)
  self.start_vlm_if_configured()
  self.create_timer(.05,self.process_commands); self.create_timer(1,self.status)
 def evt(self,name,**data): self.mqtt.publish(topic('evt'),json.dumps({'agent':agent,'name':name,'data':data}),qos=1)
 def on_image(self,msg): self.image=msg
 def on_info(self,msg): self.info=msg
 def on_aruco(self,msg): self.aruco_pose=msg.pose.pose; self.aruco_time=time.monotonic()
 def enemy_track(self,enabled):
  """Toggle ArUco-based detection of enemy rovers (markers.enemy_ids)."""
  if MarkerArray is None:
   raise ValueError('enemy tracking unavailable: aruco_det_loc is not installed')
  self.enemy_tracking=not self.enemy_tracking if enabled is None else bool(enabled)
  self.evt('result',command='enemy_track',text='enemy tracking '+('on' if self.enemy_tracking else 'off'),success=True,enabled=self.enemy_tracking)
 def on_enemy_markers(self,msg):
  """Project the largest qualifying enemy marker into a field cell."""
  if not self.enemy_tracking or self.info is None: return
  now=time.monotonic()
  if now-self.enemy_emit_at<float(cfg.get('enemy_cooldown_s',1.0)): return
  for marker in getattr(msg,'markers',[]):
   if ENEMY_IDS and int(marker.id) not in ENEMY_IDS: continue
   if corner_area(marker)<float(cfg.get('enemy_min_area_px',40.0)): continue
   corners=getattr(marker,'corners',None)
   if not corners or len(corners)<3: continue
   px=sum(c.x for c in corners)/len(corners); py=sum(c.y for c in corners)/len(corners)
   k=self.info.k
   if not k or k[0]<=0.0 or k[4]<=0.0: continue
   ray=((px-k[2])/k[0],(py-k[5])/k[4],1.0)
   try: tr=self.tf.lookup_transform(cfg.get('map_frame','map'),cfg.get('camera_frame','camera_optical_1'),rclpy.time.Time())
   except Exception: continue
   t=tr.transform.translation
   if abs(ray[2])<1e-6: continue
   cell=map_to_cell(t.x+ray[0]*(-t.z/ray[2]),t.y+ray[1]*(-t.z/ray[2]))
   if cell is None: continue
   self.enemy_emit_at=now
   self.evt('enemy',cell=cell,marker_id=int(marker.id),time=time.time())
   self.get_logger().info(f'enemy marker={marker.id} -> cell {cell}')
 def on_mqtt(self,_t,raw):
  try: self.commands.put_nowait(json.loads(raw))
  except: return
 def process_commands(self):
  """Run Sverk action calls only in the ROS executor thread.

  Calling them in the MQTT background thread mutates rclpy action waitables
  while spin() owns its wait-set, which crashes Humble with an out-of-bounds
  action-client index.
  """
  try: self.command(self.commands.get_nowait())
  except queue.Empty: pass
 def command(self,cmd):
  name,args=cmd.get('name'),cmd.get('args',{}); self.evt('accepted',text=name)
  try:
   if name=='check_cheburashka': self.check(bool(args.get('debug')))
   elif name=='set_debug': self.debug_enabled=bool(args.get('debug')); self.evt('result',text='vision stream '+('enabled' if self.debug_enabled else 'disabled'),success=True)
   elif name=='vision_debug': self.check(True)
   elif name=='takeoff': self.takeoff()
   elif name=='land':
    response=self.api.control.land(timeout=15); self.result(name,bool(response.success),str(response.message))
   elif name=='emergency_stop': self.api.control.emergency_stop(land=False); self.evt('result',text='emergency stop sent',success=True)
   elif name=='goto': self.goto_cell(args['cell'])
   elif name=='navigation_diagnostics': self.navigation_diagnostics()
   elif name=='enemy_track': self.enemy_track(bool(args.get('enabled')))
   else: self.evt('result',text='unknown command '+str(name),success=False)
  except Exception as e: self.evt('result',text=str(e),success=False)
 def result(self,name,success,message): self.evt('result',command=name,text=f'{name}: {message}',success=bool(success))
 def start_vlm_if_configured(self):
  """Start the local action server once; missing models are reported clearly."""
  if not cfg.get('vlm_autostart',True): return
  if self.vlm_client is None:
   self.evt('vlm',success=False,text='VLM package is unavailable; run the one-time VLM setup')
   return
  model=cfg.get('vlm_model_path','/home/sverk/vlm_models/SmolVLM-256M-Instruct-Q4_K_M.gguf')
  mmproj=cfg.get('vlm_mmproj_path','/home/sverk/vlm_models/mmproj-SmolVLM-256M-Instruct-f16.gguf')
  llama=cfg.get('vlm_llama_cli','/home/sverk/llama.cpp/build/bin/llama-cli')
  # A failed interrupted download leaves a zero-byte file; it is not a model.
  missing=[p for p in (model,mmproj,llama) if not os.path.isfile(p) or os.path.getsize(p)==0]
  if missing:
   self.evt('vlm',success=False,text='VLM not started; missing: '+', '.join(missing))
   return
  if self.vlm_client.server_is_ready():
   self.evt('vlm',success=True,text='VLM action server is already running'); return
  log=open('/tmp/air_watch_vlm.log','a',buffering=1)
  # Binary detection must finish quickly: 2 generated tokens are enough for
  # YES/NO, and 512 context tokens avoid allocating an unused 2048-token KV cache.
  command=['ros2','launch','vlm_perception','vlm.launch.py',f'model:={model}',f'mmproj:={mmproj}',f'llama_bin:={llama}',f"ctx:={int(cfg.get('vlm_ctx',512))}",f"max_tok:={int(cfg.get('vlm_max_tokens',2))}",f"timeout:={float(cfg.get('vlm_timeout_s',75.0))}",f"system_prompt:={cfg.get('vlm_system_prompt','Answer only YES or NO.')}" ]
  environment=os.environ.copy()
  environment['PYTHONPATH']='/usr/lib/python3/dist-packages'+(':'+environment['PYTHONPATH'] if environment.get('PYTHONPATH') else '')
  self.vlm_process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=environment)
  self.evt('vlm',success=True,text='VLM server starting on drone')
 def blue_light(self):
  """Turn LEDs blue for five seconds without blocking camera or ROS work."""
  if self.led_client is None or not self.led_client.service_is_ready():
   self.evt('light',text='blue light skipped: LED service is unavailable')
   return
  request=SetLEDEffect.Request(); request.effect='fill'; request.r=0; request.g=0; request.b=255
  self.led_client.call_async(request)
  self.evt('light',text='blue light on for 5 s')
  if self._blue_timer is not None:
   self._blue_timer.cancel()
  self._blue_timer=self.create_timer(5.0,self.blue_light_off)
 def blue_light_off(self):
  # This method is called by the one-shot timer created in blue_light.
  if self._blue_timer is not None:
   self._blue_timer.cancel(); self._blue_timer=None
  if self.led_client is not None and self.led_client.service_is_ready():
   request=SetLEDEffect.Request(); request.effect='fill'; request.r=0; request.g=0; request.b=0
   self.led_client.call_async(request)
  self.evt('light',text='orange light off')
 def request_vlm(self,image):
  """Ask the onboard VLM once per detection burst; completion is asynchronous."""
  now=time.monotonic()
  if self.vlm_busy or now-self.vlm_last_request<float(cfg.get('vlm_cooldown_s',5.0)): return
  self.vlm_last_request=now
  if self.vlm_client is None:
   self.evt('vlm',success=False,text='VLM unavailable: vlm_interfaces is not installed')
   return
  if not self.vlm_client.server_is_ready():
   self.evt('vlm',success=False,text='VLM unavailable: action /vlm/query is not running')
   return
  goal=VlmQuery.Goal(); goal.image=image; goal.query=cfg.get('vlm_prompt','Опиши найденный объект кратко: что это и какие заметны признаки?'); goal.use_npu_vision=bool(cfg.get('vlm_use_npu_vision',False))
  self.vlm_busy=True; self.evt('vlm',success=True,text='frame sent to VLM; checking for brown plush toy')
  future=self.vlm_client.send_goal_async(goal,feedback_callback=self.on_vlm_feedback)
  future.add_done_callback(self.on_vlm_goal)
 def on_vlm_feedback(self,feedback):
  stage=feedback.feedback.stage
  self.evt('vlm_progress',text='VLM: '+stage)
 def on_vlm_goal(self,future):
  try:
   handle=future.result()
   if not handle.accepted:
    self.vlm_busy=False; self.evt('vlm',success=False,text='VLM rejected the frame (busy)'); return
   handle.get_result_async().add_done_callback(self.on_vlm_result)
  except Exception as exc:
   self.vlm_busy=False; self.evt('vlm',success=False,text='VLM request error: '+str(exc))
 def on_vlm_result(self,future):
  self.vlm_busy=False
  try:
   result=future.result().result
   if result.success:
    self.evt('vlm',success=True,text=result.response,inference_ms=round(float(result.inference_time_ms),1))
   else: self.evt('vlm',success=False,text='VLM failed: '+result.error_msg)
  except Exception as exc: self.evt('vlm',success=False,text='VLM result error: '+str(exc))
 def navigation_diagnostics(self):
  """Report the live coordinate pair; no flight command is sent."""
  start=self.api.control.get_telemetry()
  data={'px4':[round(float(start.x),3),round(float(start.y),3),round(float(start.z),3),round(float(start.yaw),3)]}
  if self.aruco_pose is not None and time.monotonic()-self.aruco_time<=1.0:
   p,q=self.aruco_pose.position,self.aruco_pose.orientation
   data['aruco']=[round(float(p.x),3),round(float(p.y),3),round(float(p.z),3),round(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),3)]
  else: data['aruco']=None
  self.evt('navigation_diagnostics',text=json.dumps(data),**data)
 def takeoff(self):
  """Climb vertically above the currently measured autopilot start point."""
  start=self.api.control.get_telemetry()
  x0,y0,yaw=float(start.x),float(start.y),float(start.yaw)
  if not all(math.isfinite(v) for v in (x0,y0,yaw)):
   raise ValueError('autopilot telemetry is invalid; takeoff cancelled')
  # Do not use navigate_wait here: it monopolizes the flight API for up to a
  # minute and makes land/emergency commands appear unresponsive.
  response=self.api.control.navigate(
   x=x0,y=y0,z=float(cfg.get('takeoff_altitude_m',1.5)),yaw=yaw,
   speed=.5,frame_id='map',auto_arm=True)
  self.result('takeoff',bool(response.success),str(response.message))
 def goto_cell(self,cell):
  """Send the centre of a field cell directly in the ArUco map frame."""
  cell=str(cell).upper()
  if len(cell)<2 or cell_to_map(cell) is None: raise ValueError('invalid cell '+cell)
  if self.aruco_pose is None or time.monotonic()-self.aruco_time > 1.0:
   raise ValueError('cannot start goto: ArUco map is not visible; wait for localization')
  current=self.aruco_pose.position; target_x,target_y=cell_to_map(cell)
  dx,dy=target_x-current.x,target_y-current.y
  distance=math.hypot(dx,dy)
  if not math.isfinite(distance) or distance>8.0: raise ValueError(f'target {cell} is {distance:.1f} m away; refusing unsafe command')
  q=self.aruco_pose.orientation
  aruco_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  nav_frame=cfg.get('navigation_frame','aruco_map')
  if not self.navigation_ready(nav_frame):
   raise ValueError(f'cannot start goto: TF {nav_frame} -> base_link is unavailable')
  altitude=float(cfg.get('takeoff_altitude_m',2.0))
  self.evt('navigation_diagnostics',text=json.dumps({'frame':nav_frame,'current_aruco':[round(current.x,2),round(current.y,2),round(current.z,2)],'target_aruco':[round(target_x,2),round(target_y,2),altitude]}))
  response=self.api.control.navigate(x=target_x,y=target_y,z=altitude,yaw=aruco_yaw,frame_id=nav_frame,speed=.35)
  self.evt('result',command='goto',text=f'goto {cell}: {response.message}; {nav_frame} target {target_x:.2f}, {target_y:.2f}, {altitude:.2f}',success=bool(response.success))
 def navigation_ready(self,nav_frame=None):
  try:
   return self.tf.can_transform(nav_frame or cfg.get('navigation_frame','aruco_map'),'base_link',rclpy.time.Time(),timeout=Duration(seconds=0.0))
  except Exception:
   return False
 def vision_params(self):
  """Area threshold, linearly interpolated from the measured height table."""
  height=1.5
  if self.aruco_pose is not None and time.monotonic()-self.aruco_time <= 1.0:
   height=abs(float(self.aruco_pose.position.z))
  points=[(.7,20000),(1.2,7800),(1.45,5200),(1.5,4000),(1.8,3700),(2.5,1000)]
  height=max(points[0][0],min(points[-1][0],height))
  for (h1,a1),(h2,a2) in zip(points,points[1:]):
   if h1<=height<=h2:
    area=a1+(a2-a1)*(height-h1)/(h2-h1); break
  # Small sensitivity increase without changing the calibrated height curve.
  area *= float(cfg.get('cheburashka_area_factor', 0.90))
  params=dict(cfg); params['cheburashka_min_area_px']=area
  return params,height,area
 def check(self,debug):
  if self.image is None or self.info is None: self.evt('cheburashka',text='camera not ready'); return
  params,height,area=self.vision_params()
  found=detect(self.bridge.imgmsg_to_cv2(self.image,'bgr8'),params,debug)
  if debug and found and found.get('debug_image') is not None:
   # Same built-in Sverk preview publication used in cv.py.
   self.api.image.publish(found['debug_image'])
  if not found or not found.get('found', True):
   self.evt('cheburashka',detected=False,text='not found');
   if debug:self.evt('vision',text='No brown/orange component passed the 5000 px² merged-area threshold.')
   return
  # Detection itself is useful to the dispatcher even if its later projection
  # to a field cell fails because a camera TF is temporarily unavailable.
  self.evt('cheburashka',detected=True,text=f'visible; area {found["area_px"]} px2',confidence=found['confidence'],area=found['area_px'])
  self.blue_light(); self.request_vlm(self.image)
  px,py=found['pixel']; k=self.info.k; ray=((px-k[2])/k[0],(py-k[5])/k[4],1.0)
  found.pop('debug_image',None)
  source_frame=self.image.header.frame_id or self.info.header.frame_id or cfg['camera_frame']
  tr=self.tf.lookup_transform(cfg['map_frame'],source_frame,rclpy.time.Time())
  q=tr.transform.rotation; tx,ty,tz=tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z
  # Camera is downward-facing; for the calibrated frame z ray is sufficient for a ground intersection.
  if abs(ray[2])<1e-6: self.evt('cheburashka',text='invalid camera ray'); return
  cell=map_to_cell(tx+ray[0]*(-tz/ray[2]),ty+ray[1]*(-tz/ray[2]))
  self.evt('cheburashka',detected=True,text=(('found in '+cell if cell else 'found, outside configured field')+f'; height {height:.2f} m, threshold {area} px2'),cell=cell or '',confidence=found['confidence'],area=found['area_px'])
  if debug:self.evt('vision',text=json.dumps(found,ensure_ascii=False))
 def debug_loop(self):
  """Continuous cv.py-style diagnostic stream while the dashboard checkbox is on."""
  if not self.debug_enabled or self.image is None: return
  try:
   params,_,_=self.vision_params()
   found=detect(self.bridge.imgmsg_to_cv2(self.image,'bgr8'),params,True)
   if found and found.get('debug_image') is not None:
    self.api.image.publish(found['debug_image'])
   # Debug stream gives visual/LED confirmation, but never spends VLM time.
   if found and found.get('found'):
    self.blue_light()
  except Exception as exc:
   self.get_logger().warn('vision stream: '+str(exc))
 def status(self):
  try:
   # Direct output of aruco_loc; stale data is never displayed as a live pose.
   if self.aruco_pose is None or time.monotonic()-self.aruco_time > 1.0: raise ValueError('ArUco pose stale')
   x,y=self.aruco_pose.position.x,self.aruco_pose.position.y
   if not math.isfinite(x) or not math.isfinite(y): raise ValueError('non-finite ArUco pose')
   cell=map_to_cell(x,y); self.evt('status',cell=cell or '',map=[round(x,2),round(y,2)],field=map_to_field(x,y),localized=True,nav_ready=self.navigation_ready())
  except Exception:
   self.evt('status',cell='',map=None,field=None,localized=False,nav_ready=False)

if __name__=='__main__':
 rclpy.init(); n=Drone()
 try: rclpy.spin(n)
 except KeyboardInterrupt: pass
 finally: n.mqtt.stop(); n.destroy_node(); rclpy.shutdown()
