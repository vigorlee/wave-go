#!/usr/bin/env python3
"""Publish the saved semantic scene as ROS 1 RViz PointCloud2 + markers.

No odometry replay, robot command, synthetic point densification, or fake live map.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]
FRAME = 'lio_map'
COLORS = {'stairs': (38,170,255), 'ramp': (180,115,250),
          'cylinder': (255,171,48), 'person': (250,70,135),
          'wall': (137,172,200), 'floor': (62,200,159),
          'region': (215,205,140), 'building': (235,241,250)}


def cloud_msg(points, rgb):
    packed = np.zeros(len(points), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    for i, name in enumerate('xyz'):
        packed[name] = points[:, i]
    c = rgb.astype(np.uint32)
    packed['rgb'] = (c[:,0] << 16) | (c[:,1] << 8) | c[:,2]
    msg = PointCloud2()
    msg.header.frame_id = FRAME
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(n, i*4, PointField.FLOAT32, 1) for i,n in enumerate(['x','y','z'])]
    msg.fields.append(PointField('rgb', 12, PointField.UINT32, 1))
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = len(points)*16
    msg.is_dense = True
    msg.data = packed.tobytes()
    return msg


def marker(ns, mid, kind, position, color=(240,240,240), alpha=1.):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
    m.pose.orientation.w = 1
    m.pose.position = Point(*map(float, position))
    m.color.r, m.color.g, m.color.b = [v/255 for v in color]
    m.color.a = alpha
    return m


def text(ns, mid, title, pos, color, size=.28):
    m = marker(ns, mid, Marker.TEXT_VIEW_FACING, pos, color)
    m.text = title
    m.scale.z = size
    return m


def lines(ns, mid, pairs, color, width=.025, alpha=.85):
    m = marker(ns, mid, Marker.LINE_LIST, [0,0,0], color, alpha)
    m.scale.x = width
    m.points = [Point(*map(float, p)) for pair in pairs for p in pair]
    return m


def box_edges(lo, hi):
    corners = [np.where([(i>>b)&1 for b in range(3)], hi, lo) for i in range(8)]
    return [(corners[i], corners[i^(1<<b)]) for i in range(8) for b in range(3) if not (i>>b)&1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=ROOT/'outputs')
    args = parser.parse_args()
    rospy.init_node('wavego_semantic_hierarchy')
    data = np.load(args.input/'semantic_cloud.npz')
    scene = json.loads((args.input/'scene_graph.json').read_text())
    path = np.load(args.input/'trajectory.npy')[:,1:]
    roi, semantic = data['roi'], data['labels'] > 0
    streams = {}
    for name, mask in [('context', roi & ~semantic), ('semantic_points', roi & semantic), ('full_cloud', np.ones(len(roi),bool))]:
        colors = data['colors'][mask].copy()
        streams[name] = cloud_msg(data['points'][mask], colors)

    boxes, structures, graph, route = [], [], [], []
    nodes = scene['nodes']
    for i,n in enumerate(nodes):
        color = COLORS.get(n['kind'], (180,180,180))
        if n['kind'] in ['stairs','ramp','cylinder','person']:
            lo, hi = np.array(n['bounds'])
            boxes.append(lines('object_boxes',i,box_edges(lo,hi),color,.025))
            pos = (lo+hi)/2
            pos[2] = hi[2] + .34 + (i%2)*.12
            boxes.append(text('object_names',i,n['label']+'?',pos,color,.40))
        if n['kind'] == 'wall':
            lo, hi = np.array(n['bounds'])
            # Plane wireframes available separately; avoid obscuring stairs.
            structures.append(lines('wall_planes',i,box_edges(lo,hi),color,.012,.28))
        if n['kind'] == 'floor':
            lo, hi = np.array(n['bounds'])
            lo[2] = hi[2] = n['z']
            structures.append(lines('floor_extents',i,box_edges(lo,hi),color,.045,.8))
            pos = [lo[0], lo[1]-.55, n['z']+.3]
            structures.append(text('floor_names',i,f'{n["id"]} | z={n["z"]:.2f} m | candidate',pos,color,.48))

    # The graph uses a disclosed schematic layout. Metric positions are stored
    # unchanged in scene_graph.json / spark_dsg.json and used by object boxes.
    display_pos = {}
    floors = [n for n in nodes if n['kind']=='floor']
    regions = [n for n in nodes if n['kind']=='region']
    display_pos['B0'] = [8., -12., 11.5]
    for j,f in enumerate(floors):
        center=-2.+j*20
        display_pos[f['id']] = [center, -12., 8.]
        subset = [r for r in regions if r['floor']==f['id']]
        for k,r in enumerate(subset):
            display_pos[r['id']] = [center+(k-(len(subset)-1)/2)*4.0, -12., 4.5]
        objects = [n for n in nodes if n.get('floor')==f['id'] and n['kind'] not in ['region','floor']]
        for k,n in enumerate(objects):
            display_pos[n['id']] = [center+(k-(len(objects)-1)/2)*1.25, -12., 1.0 if n['kind']!='wall' else -.5]
    for i,n in enumerate(nodes):
        if n['id'] not in display_pos:
            continue
        pos=display_pos[n['id']]
        color=COLORS[n['kind']]
        sphere=marker('graph_nodes',i,Marker.SPHERE,pos,color)
        sphere.scale.x=sphere.scale.y=sphere.scale.z=.28 if n['kind']=='wall' else .45
        graph.append(sphere)
        title = n['id']+' '+n['kind'] if n['kind']!='wall' else n['id']
        if n['kind']=='floor': title += ' (candidate)'
        graph.append(text('graph_labels',i,title,[pos[0],pos[1],pos[2]+.45],color,.32))
    for i,e in enumerate(scene['edges']):
        if e['source'] in display_pos and e['target'] in display_pos:
            color=(38,170,255) if e['relation']=='stairs_candidate' else (140,159,185)
            graph.append(lines('graph_edges',i,[(display_pos[e['source']],display_pos[e['target']])],color,.025,.7))
    graph.append(text('graph_info',0,'SCENE GRAPH | schematic layout | floor / region hypotheses',
                      [8,-12,13.],(225,234,247),.45))

    sampled=path[::4]
    route.append(lines('recorded_lio_trajectory',0,list(zip(sampled[:-1],sampled[1:])),(255,235,135),.048,.9))
    route.append(text('trajectory_label',0,'Recorded LIO trajectory',path[0]+[0,0,.65],(255,235,135),.3))
    for name,markers in [('objects',boxes),('structures',structures),('scene_graph',graph),('trajectory',route)]:
        streams[name]=MarkerArray(markers=markers)
    pubs={name:rospy.Publisher('/semantic_hierarchy/'+name,type(msg),queue_size=1,latch=True) for name,msg in streams.items()}
    rospy.loginfo('Saved map visualization: %s points, %s nodes; frame=%s',len(data['points']),len(nodes),FRAME)
    rate=rospy.Rate(.5)
    while not rospy.is_shutdown():
        for name,msg in streams.items():
            if hasattr(msg,'header'): msg.header.stamp=rospy.Time.now()
            else:
                for m in msg.markers: m.header.stamp=rospy.Time.now()
            pubs[name].publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
