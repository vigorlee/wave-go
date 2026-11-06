#!/usr/bin/python3
"""OfficeWorld coordinates with unchanged terrain evidence thresholds."""
import os,subprocess
from dataclasses import replace
from pathlib import Path
import go2w_continuous_course as base
ROOT=Path(__file__).resolve().parents[1]
class OfficeCourse(base.ContinuousCourse):
 def set_nav_mode(self,mode):
  if mode=='avoid' and self.stage_index>=2 and not getattr(self,'office_stairs_closed',False):
   # Stairs are traversable along the stair route, but their side walls are
   # obstacles once the robot has descended. Avoid diagonal shortcuts.
   command=['ros2','service','call','/map_server/load_map','nav2_msgs/srv/LoadMap','{map_url: "'+str(ROOT/'config/go2w_office_avoid.yaml')+'"}']
   result=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=15)
   if result.returncode or 'result=0' not in result.stdout:
    self.get_logger().error('Could not activate ground-navigation map: '+result.stdout[-500:]);return False
   self.office_stairs_closed=True
   self.get_logger().info('Office ground-navigation map activated: stair sides excluded from shortcut planning')
  return super().set_nav_mode(mode)
 def update_ramp_physics(self,pose):
  # Only actual poses inside the translated ramp room update the ramp test.
  if 15.7<=pose.x<=19.0:
   super().update_ramp_physics(replace(pose,x=pose.x-17.2,y=pose.y+18.5))
 def continuous_stage_reached(self,stage,pose,now):
  if stage.stage_id in ('ramp_up_entry','ramp_up_crest','ramp_down_entry','ramp_down_exit'):
   if not 15.7<=pose.x<=19.0:return False,'outside_physical_ramp_room'
   return super().continuous_stage_reached(stage,replace(pose,x=pose.x-17.2,y=pose.y+18.5),now)
  return super().continuous_stage_reached(stage,pose,now)
if __name__=='__main__':
 settings={'COSMOS_VLN_CONTINUOUS_PASS_RADIUS':'.70','COSMOS_VLN_ROUTES_FILE':str(ROOT/'config/go2w_office_continuous_route.json'),'GO2W_COURSE_ROUTE_ID':'officeworld_migrated_course','GO2W_COURSE_SCENE':str(ROOT/'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_course.xml'),'GO2W_COURSE_START_X':'-3.2','GO2W_COURSE_START_Y':'-6.0','COSMOS_VLN_STAIR_START_MIN_Y':'-6.3','COSMOS_VLN_STAIR_START_MAX_Y':'-5.55','COSMOS_VLN_STAIR_PLATFORM_MIN_Y':'-1.8','COSMOS_VLN_STAIR_DOWN_START_MIN_Y':'-1.8','COSMOS_VLN_STAIR_DOWN_PLATFORM_MIN_Y':'3.6','COSMOS_VLN_MISSION_TIMEOUT_SEC':'600'}
 for k,v in settings.items():os.environ.setdefault(k,v)
 base.ContinuousCourse=OfficeCourse;base.main()
