#!/usr/bin/env python3
"""White-background instance / height-map visualization of recorded LIO scans."""
import colorsys
import json
from pathlib import Path
import threading
import numpy as np
import rospy
from std_msgs.msg import Float64,String
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import PointCloud2
from publish_scene import cloud_msg,marker,lines,text,box_edges
from metric_graph_markers import graph_markers
from academic_palette import height_colors, paper_height_colors, SEMANTIC
from scene_profile import DATA_ROOT,CHINESE
from semantic_labels_cn import KIND_ZH

ROOT=Path(__file__).resolve().parents[1]
OUT=DATA_ROOT/'outputs/reference_demo'
CN=dict(KIND_ZH, floor='楼层地面')
PALETTE={'stairs':(70,125,225),'ramp':(180,105,210),'cylinder':(230,155,65),
         'wall':(85,170,145),'floor':(100,155,205),'region':(185,155,80),'person':(220,80,120)}


def legacy_height_colors(z):
    """Fixed metric elevation color scale, unaffected by replay progress."""
    anchors=np.array([[52,105,183],[56,182,175],[125,198,86],[245,212,63],
                      [238,135,50],[192,52,51]],float)
    t=np.clip((z+.6)/6.,0,1)*(len(anchors)-1)
    low=np.minimum(t.astype(int),len(anchors)-2)
    fraction=t-low
    return (anchors[low]*(1-fraction[:,None])+anchors[low+1]*fraction[:,None]).astype(np.uint8)


def main():
    rospy.init_node('wavego_reference_semantic_demo')
    data=np.load(OUT/'mapping_replay.npz')
    p,seen,labels=data['points'],data['first_seen'],data['labels']
    native=np.load(OUT/'model_semantics.npz')
    native_label=native['labels']
    display_colors=SEMANTIC[native_label]
    report=json.loads((OUT/'replay_report.json').read_text())
    scene=json.loads((DATA_ROOT/'outputs/scene_graph.json').read_text())
    trajectory=np.load(DATA_ROOT/'outputs/trajectory.npy')
    nodes=[n for n in scene['nodes'] if n['kind'] in CN]
    indices={}
    model_indices={}
    annotations=[]
    for n in nodes:
        lo,hi=np.array(n['bounds'])
        match=np.all((p>=lo-.06)&(p<=hi+.06),axis=1)
        if n['kind'] in ['stairs','ramp','cylinder','person','wall']:
            match&=labels==n['semantic_label']
        elif n['kind']=='floor':
            match&=(labels==6)&(abs(p[:,2]-n['z'])<.24)
        else:
            match&=labels==6
        ids=np.flatnonzero(match)
        indices[n['id']]=ids
        if n['kind'] in ['stairs','ramp','cylinder','person']:
            model_indices[n['id']]=np.flatnonzero(np.all((p>=lo-.06)&(p<=hi+.06),axis=1)&(native_label==n['semantic_label']))
        annotations.append(dict(**n,label_zh=CN[n['kind']],associated_dense_points=len(ids),
                                first_observation_s=float(seen[ids[0]]-seen[0]) if len(ids) else None))
    (OUT/'semantic_annotations.json').write_text(json.dumps(annotations,ensure_ascii=False,indent=2))
    hcolors=height_colors(p[:,2])
    papercolors=paper_height_colors(p[:,2])
    # Cutaway is a display-only mask: all measured XYZ remain in the PCD.
    floor_z=[n['z'] for n in nodes if n['kind']=='floor']
    cutaway=np.zeros(len(p),bool)
    for z in floor_z:cutaway|=(p[:,2]>z-.30)&(p[:,2]<z+1.25)
    for n in nodes:
        if n['kind'] in ['stairs','ramp']:
            lo,hi=np.array(n['bounds']);cutaway|=np.all((p>=lo)&(p<=hi),axis=1)
    colors=np.full((len(p),3),180,np.uint8)
    for name,color in PALETTE.items():
        if name in scene['ontology']:colors[labels==scene['ontology'].index(name)]=color
    # Save a dense labelled PCD independent of the viewer.
    packed=np.empty(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('label','u1'),('evidence','u1')])
    for i,k in enumerate('xyz'):packed[k]=p[:,i]
    rgb=colors.astype(np.uint32)
    packed['rgb']=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
    packed['label']=labels
    packed['evidence']=data['evidence']
    header=('VERSION .7\nFIELDS x y z rgb label evidence\nSIZE 4 4 4 4 1 1\n'
            f'TYPE F F F U U U\nCOUNT 1 1 1 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\n'
            f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n')
    (OUT/'dense_semantic_map.pcd').write_bytes(header.encode()+packed.tobytes())
    pubs={k:rospy.Publisher('/reference_map/'+k,t,queue_size=1,latch=True) for k,t in
          [('height_points',PointCloud2),('paper_height_points',PointCloud2),('height_cutaway',PointCloud2),('semantic_points',PointCloud2),
           ('model_semantic_points',PointCloud2),('model_context',PointCloud2),('model_boxes',MarkerArray),
           ('model_labels',MarkerArray),
           ('metric_graph',MarkerArray),('metric_graph_labels',MarkerArray),
           ('instances',MarkerArray),('labels',MarkerArray),('trajectory',MarkerArray),('status',String)]}
    floor_file=DATA_ROOT/'outputs/three_representations/hovsg_floor_probe/floor_labels.npz'
    if floor_file.exists():
        # Unmodified upstream floor extraction, shown as a static final-map
        # comparison; distinct from model class colors and replay predictions.
        floor_ids=np.load(floor_file)['labels']
        assert len(floor_ids)==len(p)
        floor_palette=np.array([[195,201,210]]+[[(64,144,202),(228,166,59),(132,165,132),(163,135,172)][i%4] for i in range(int(floor_ids.max()))],np.uint8)
        floor_cloud=cloud_msg(p,floor_palette[floor_ids])
        floor_cloud.header.stamp=rospy.Time.now()
        pubs['hovsg_floors']=rospy.Publisher('/reference_map/hovsg_floors',PointCloud2,queue_size=1,latch=True)
        pubs['hovsg_floors'].publish(floor_cloud)
    feature_file=DATA_ROOT/'outputs/academic_upgrade/utonia_features.npz'
    if feature_file.exists():
        feature_colors=np.load(feature_file)['colors']
        assert feature_colors.shape==(len(p),3)
        feature_cloud=cloud_msg(p,feature_colors)
        feature_cloud.header.stamp=rospy.Time.now()
        pubs['utonia_features']=rospy.Publisher('/reference_map/utonia_features',PointCloud2,queue_size=1,latch=True)
        pubs['utonia_features'].publish(feature_cloud)
    lock=threading.Lock()
    request=[report['duration_seconds']]
    style=['paper']
    def representation(m):
        obj=json.loads(m.data)
        with lock:style[0]=obj.get('style','paper')
    rospy.Subscriber('/reference_map/representation',String,representation,queue_size=1)
    def seek(m):
        with lock:request[0]=float(np.clip(m.data,0,report['duration_seconds']))
    rospy.Subscriber('/reference_map/seek',Float64,seek,queue_size=1)
    last=None
    rate=rospy.Rate(4)
    while not rospy.is_shutdown():
        with lock:elapsed=request[0];dark=style[0]=='fastlio'
        if last is not None and abs(last[0]-elapsed)<.005 and last[1]==dark:
            rate.sleep();continue
        last=(elapsed,dark)
        timestamp=report['first_stamp']+elapsed
        count=np.searchsorted(seen,timestamp,side='right')
        for name,col in [('height_points',hcolors),('paper_height_points',papercolors),('semantic_points',colors)]:
            msg=cloud_msg(p[:count],col[:count])
            msg.header.stamp=rospy.Time.now()
            pubs[name].publish(msg)
        visible_cutaway=cutaway[:count]
        msg=cloud_msg(p[:count][visible_cutaway],hcolors[:count][visible_cutaway])
        msg.header.stamp=rospy.Time.now();pubs['height_cutaway'].publish(msg)
        # Unknown geometry and coloured model predictions are distinct layers,
        # but retain exactly the same metric coordinate frame in the single map.
        labelled=native_label[:count]>0
        for name,mask in [('model_semantic_points',labelled),('model_context',~labelled)]:
            msg=cloud_msg(p[:count][mask],display_colors[:count][mask])
            msg.header.stamp=rospy.Time.now();pubs[name].publish(msg)
        shapes=[Marker(action=Marker.DELETEALL)]
        labels_mark=[Marker(action=Marker.DELETEALL)]
        visible=[]
        model_boxes=[Marker(action=Marker.DELETEALL)]
        model_labels=[Marker(action=Marker.DELETEALL)]
        for i,n in enumerate(nodes):
            ids=indices[n['id']]
            ids=ids[:np.searchsorted(ids,count)]
            if len(ids)<15:continue
            q=p[ids]
            lo,hi=np.quantile(q,[.02,.98],axis=0)
            if n['kind'] in ['floor','region']:
                lo[2]=hi[2]=float(n.get('z',(lo[2]+hi[2])/2))
            center=(lo+hi)/2
            size=np.maximum(hi-lo,.035)
            base=np.array(PALETTE[n['kind']])
            color=tuple(np.clip(base+(i%3-1)*12,0,255).astype(int))
            alpha=.20 if n['kind'] not in ['floor','region'] else .045
            cube=marker('instance_faces',i,Marker.CUBE,center,color,alpha)
            cube.scale.x,cube.scale.y,cube.scale.z=map(float,size)
            shapes.append(cube)
            shapes.append(lines('instance_edges',i,box_edges(lo,hi),(60,68,80),.017,.82))
            top=center.copy()
            top[2]=hi[2]+.35
            # All instances have a persistent ID and an explicit candidate label.
            name={'floor':'楼层*','region':'区域*','wall':'墙面*'}.get(n['kind'],n.get('label_zh',CN.get(n['kind'],n['kind']))+'?') if CHINESE else {'floor':'floor*','region':'region*','wall':'wall*'}.get(n['kind'],n['kind']+'?')
            labels_mark.append(text('instance_names',i,n['id']+' '+name,top,(25,32,45),.40))
            shapes.append(lines('label_leaders',i,[(center+[0,0,size[2]/2],top)],(90,100,115),.01,.55))
            visible.append(n['id'])
            # These are visualization clusters of predicted object classes;
            # geometric room/floor/wall hypotheses are not model detections.
            if n['kind'] in ['stairs','ramp','cylinder','person']:
                native_ids=model_indices[n['id']]
                native_ids=native_ids[:np.searchsorted(native_ids,count)]
                if len(native_ids)<15:continue
                native_lo,native_hi=np.quantile(p[native_ids],[.02,.98],axis=0)
                klass=n['semantic_label']
                palette=SEMANTIC[klass]
                model_boxes.append(lines('model_object_outline',i,box_edges(native_lo,native_hi),tuple(palette),.035,1.))
                top=(native_lo+native_hi)/2;top[2]=native_hi[2]+.35
                label_color=np.minimum(palette.astype(int)+65,245) if dark else (palette*.60).astype(int)
                model_labels.append(text('model_object_label',i,n['id']+' '+(n.get('label_zh',CN.get(n['kind'],n['kind'])) if CHINESE else n['kind'])+'?',top,tuple(label_color),.42))
        pubs['instances'].publish(MarkerArray(markers=shapes))
        pubs['labels'].publish(MarkerArray(markers=labels_mark))
        pubs['model_boxes'].publish(MarkerArray(markers=model_boxes))
        pubs['model_labels'].publish(MarkerArray(markers=model_labels))
        graph,graph_labels=graph_markers(scene,set(visible),dark=dark)
        pubs['metric_graph'].publish(graph)
        pubs['metric_graph_labels'].publish(graph_labels)
        poses=trajectory[trajectory[:,0]<=timestamp,1:]
        pathmarks=[Marker(action=Marker.DELETEALL)]
        if len(poses):
            subsampled=poses[::3]
            pathmarks.append(lines('recorded_path',0,list(zip(subsampled[:-1],subsampled[1:])),(45,52,163),.055,1.))
            robot=marker('current_lio_pose',0,Marker.SPHERE,poses[-1],(40,60,160))
            robot.scale.x=robot.scale.y=robot.scale.z=.22
            pathmarks.append(robot)
        pubs['trajectory'].publish(MarkerArray(markers=pathmarks))
        pubs['status'].publish(String(data=json.dumps(dict(elapsed=elapsed,duration=report['duration_seconds'],
                                  points=int(count),instances=visible,source='recorded scan accumulation',
                                  semantics='Grounding DINO + SAM2 model classes projected to LIO map; Chinese labels are display translations'))))
        rate.sleep()


if __name__=='__main__':main()
