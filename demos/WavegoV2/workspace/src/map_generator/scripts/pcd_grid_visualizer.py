#!/usr/bin/env python3
"""Project the package's PCD cloud into a height-sliced occupancy grid.
Unknown stays unknown: occupied endpoints do not imply free space along rays.
"""
from pathlib import Path
import json
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
rospy.init_node('pcd_grid_visualizer')
r=float(rospy.get_param('~resolution',0.10));zmin=float(rospy.get_param('~obstacle_z_min',-0.40));zmax=float(rospy.get_param('~obstacle_z_max',1.30))
fmin=float(rospy.get_param('~floor_z_min',-0.95));fmax=float(rospy.get_param('~floor_z_max',-0.45))
out=Path(rospy.get_param('~output_dir'));out.mkdir(parents=True,exist_ok=True)
pubs={n:rospy.Publisher('/map_generator/'+n,OccupancyGrid,queue_size=1,latch=True) for n in ['grid','grid_inflated']}
voxpub=rospy.Publisher('/map_generator/voxel_cloud',PointCloud2,queue_size=1,latch=True)
m=rospy.wait_for_message('/map_generator/global_cloud',PointCloud2,timeout=30)
off={f.name:f.offset for f in m.fields}
a=np.ndarray((m.width*m.height,),dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[off[n] for n in ['x','y','z']],'itemsize':m.point_step}),buffer=m.data)
xyz=np.column_stack([a[n] for n in ['x','y','z']]);xyz=xyz[np.isfinite(xyz).all(axis=1)]
lo=np.floor(xyz[:,:2].min(axis=0)/r)*r-r;hi=np.ceil(xyz[:,:2].max(axis=0)/r)*r+r
w,h=np.ceil((hi-lo)/r).astype(int);ij=np.floor((xyz[:,:2]-lo)/r).astype(int);idx=ij[:,1]*w+ij[:,0]
floor=(xyz[:,2]>=fmin)&(xyz[:,2]<=fmax);obs=(xyz[:,2]>=zmin)&(xyz[:,2]<=zmax)
fc=np.bincount(idx[floor],minlength=w*h).reshape(h,w);oc=np.bincount(idx[obs],minlength=w*h).reshape(h,w)
grid=np.full((h,w),-1,dtype=np.int8);grid[fc>=2]=0;grid[oc>=2]=100
rad=float(rospy.get_param('~inflation_radius',0.3));k=int(np.ceil(rad/r));yy,xx=np.mgrid[-k:k+1,-k:k+1];mask=(xx*r)**2+(yy*r)**2<=rad**2+1e-9
expanded=np.zeros_like(grid,dtype=bool)
padded=np.pad(grid==100,k)
for dy,dx in np.argwhere(mask):
 expanded |= padded[dy:dy+h,dx:dx+w]
inflated=grid.copy();inflated[expanded]=100
for n,g in [('grid',grid),('grid_inflated',inflated)]:
 msg=OccupancyGrid();msg.header.frame_id=m.header.frame_id;msg.header.stamp=rospy.Time.now();msg.info.resolution=r;msg.info.width=int(w);msg.info.height=int(h);msg.info.origin.position.x=float(lo[0]);msg.info.origin.position.y=float(lo[1]);msg.info.origin.orientation.w=1;msg.data=g.ravel().tolist();pubs[n].publish(msg)
 pixels=np.full(g.shape,205,dtype=np.uint8);pixels[g==0]=254;pixels[g==100]=0
 (out/(n+'.pgm')).write_bytes(('P5\n%d %d\n255\n'%(w,h)).encode()+pixels[::-1].tobytes())
 (out/(n+'.yaml')).write_text(f'image: {n}.pgm\nresolution: {r}\norigin: [{lo[0]}, {lo[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
vr=float(rospy.get_param('~voxel_resolution',0.05))
centers=((np.unique(np.floor(xyz/vr).astype(np.int32),axis=0)+0.5)*vr).astype('<f4')
np.savez_compressed(out/'voxel_grid_3d.npz',centers=centers,resolution=vr,frame=m.header.frame_id)
v=PointCloud2();v.header=m.header;v.height=1;v.width=len(centers);v.fields=[PointField(n,i*4,7,1) for i,n in enumerate(['x','y','z'])];v.point_step=12;v.row_step=v.width*12;v.data=centers.tobytes();v.is_dense=True;voxpub.publish(v)
report=dict(input_points=len(xyz),voxels=len(centers),resolution_m=r,voxel_resolution_m=vr,frame=m.header.frame_id,width=int(w),height=int(h),obstacle_z=[zmin,zmax],floor_z=[fmin,fmax],occupied_cells=int((grid==100).sum()),floor_supported_cells=int((grid==0).sum()),unknown_cells=int((grid<0).sum()),inflation_radius_m=rad,note='Static height-slice projection; free labels are floor-supported, not ray-traced or certified traversable. Multi-floor geometry may overlap.')
(out/'grid_report.json').write_text(json.dumps(report,indent=2));rospy.loginfo(str(report));rospy.spin()
