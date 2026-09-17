#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
rospy.init_node('adaptive_voxel_display',anonymous=True)
prefix=rospy.get_param('~topic_prefix','/map_generator/adaptive')
a=np.load(rospy.get_param('~map_file'));pubs=[]
for size in np.unique(a['sizes']):
    sel=a['sizes']==size;xyz=a['centers'][sel];rgb=a['rgb'][sel].astype('u4')
    data=np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    for j,n in enumerate(['x','y','z']):data[n]=xyz[:,j]
    data['rgb']=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
    m=PointCloud2();m.header.frame_id=str(a['frame']);m.header.stamp=rospy.Time.now();m.height=1;m.width=len(xyz);m.fields=[PointField(n,i*4,7,1) for i,n in enumerate(['x','y','z','rgb'])];m.point_step=16;m.row_step=16*m.width;m.is_dense=True;m.data=data.tobytes()
    p=rospy.Publisher(prefix+'_%02dcm'%round(float(size)*100),PointCloud2,queue_size=1,latch=True);pubs.append(p);p.publish(m)
rospy.loginfo('Adaptive voxel levels published with shared height RGB colors')
rospy.spin()
