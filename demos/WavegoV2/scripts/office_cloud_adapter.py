import rospy,numpy as np
from sensor_msgs.msg import PointCloud2,PointField
rospy.init_node('office_cloud_adapter')
pub=rospy.Publisher('/office/fastlio_input',PointCloud2,queue_size=5)
# The simulator supplies instantaneous scans with zero per-point timestamps.
# Ouster-compatible fields select a parser that preserves zero offsets.
names=['x','y','z','intensity','t','reflectivity','ring','ambient','range']
formats=['<f4']*4+['<u4','<u2','u1','<u2','<u4']
dtype=np.dtype(list(zip(names,formats)))
codes=[7,7,7,7,6,4,2,4,6]
fields=[PointField(n,dtype.fields[n][1],c,1) for n,c in zip(names,codes)]
def cloud(m):
 src=np.ndarray((m.width*m.height,),dtype=np.dtype({'names':['x','y','z','intensity','line'],'formats':['<f4']*4+['u1'],'offsets':[0,4,8,12,17],'itemsize':m.point_step}),buffer=m.data)
 xyz=np.column_stack([src[n] for n in ['x','y','z']])
 valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)<35.0)
 src=src[valid]
 a=np.zeros(len(src),dtype=dtype)
 for n in names[:4]:a[n]=src[n]
 a['ring']=src['line'];a['range']=(np.sqrt(a['x']**2+a['y']**2+a['z']**2)*1000).astype('u4')
 out=PointCloud2(header=m.header,height=1,width=len(a),fields=fields,is_bigendian=False,point_step=dtype.itemsize,row_step=len(a)*dtype.itemsize,data=a.tobytes(),is_dense=False)
 pub.publish(out)
rospy.Subscriber('/livox/lidar_raw',PointCloud2,cloud,queue_size=10,buff_size=16000000)
rospy.spin()
