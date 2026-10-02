#!/usr/bin/env python3
"""Publish one imported LAZ map and its geometric graph to ROS 1 RViz."""
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray

from imported_923_io import read_pcd
from publish_scene import cloud_msg, marker, lines, text, box_edges

FRAME = 'imported_923'
PALETTE = [(68, 121, 176), (104, 164, 141), (197, 148, 78),
           (151, 127, 187), (190, 113, 125), (101, 156, 181)]


def frame_cloud(points, colors):
    msg = cloud_msg(points, colors)
    msg.header.frame_id = FRAME
    msg.header.stamp = rospy.Time.now()
    return msg


def frame_markers(markers):
    for item in markers:
        item.header.frame_id = FRAME
        item.header.stamp = rospy.Time.now()
    return MarkerArray(markers=markers)


def height_palette(z):
    """Muted blue/teal/gold measured-Z palette, clipped to robust map bounds."""
    anchors = np.array([[-1.25, 62, 104, 164], [0.0, 76, 151, 157],
                        [1.55, 207, 180, 112]], dtype=float)
    return np.column_stack([np.interp(z, anchors[:, 0], anchors[:, channel])
                            for channel in (1, 2, 3)]).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    args = parser.parse_args()
    rospy.init_node('wavego_imported_923_publisher')
    data = args.data
    raw = read_pcd(data / '923_rgb_intensity.pcd')
    points = np.stack([raw[name] for name in 'xyz'], axis=1)
    rgb = np.column_stack(((raw['rgb'] >> 16) & 255,
                           (raw['rgb'] >> 8) & 255,
                           raw['rgb'] & 255)).astype(np.uint8)
    ids = np.load(data / 'cluster_ids.npy')
    graph = json.loads((data / 'geometry_graph.json').read_text())
    if len(ids) != len(points) or graph['point_count'] != len(points):
        raise ValueError('Graph and cloud point counts do not match')
    selected = ids > 0
    cluster_colors = np.asarray(PALETTE, dtype=np.uint8)[ids[selected]-1]
    context_colors = np.full((int((~selected).sum()), 3), (130, 142, 157), np.uint8)
    clouds = {'rgb': frame_cloud(points, rgb),
              'height': frame_cloud(points, height_palette(points[:, 2])),
              'components': frame_cloud(points[selected], cluster_colors),
              'context': frame_cloud(points[~selected], context_colors)}

    boxes, graph_shapes, box_labels, graph_labels = ([Marker(action=Marker.DELETEALL)] for _ in range(4))
    nodes = {n['id']: n for n in graph['nodes']}
    for index, node in enumerate(graph['nodes']):
        pos = np.asarray(node['position'])
        if index == 0:
            color = (215, 225, 236)
            size = .32
        else:
            color = PALETTE[index-1]
            size = .29
            lo, hi = np.asarray(node['bounds'])
            boxes.append(lines('geometry_bounds', index, box_edges(lo, hi), color, .023, .9))
            top = (lo + hi)/2
            top[2] = hi[2] + .2
            box_labels.append(text('geometry_box_label', index,
                                   f'{node["id"]} {node["label_zh"]}', top, color, .38))
        sphere = marker('geometry_graph_nodes', index, Marker.SPHERE, pos, color)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = size
        graph_shapes.append(sphere)
        graph_labels.append(text('geometry_graph_label', index,
                                 f'{node["id"]} {node["label_zh"]}', pos + [0, 0, .28], color, .35))
    for index, edge in enumerate(graph['edges']):
        a = nodes[edge['source']]['position']
        b = nodes[edge['target']]['position']
        graph_shapes.append(lines('same_source_cloud', index, [(a, b)], (232, 203, 131), .055, .95))
    streams = dict(clouds, boxes=frame_markers(boxes), graph=frame_markers(graph_shapes),
                   box_labels=frame_markers(box_labels), graph_labels=frame_markers(graph_labels))
    publishers = {name: rospy.Publisher('/imported_923/' + name, type(message), queue_size=1, latch=True)
                  for name, message in streams.items()}
    for name, message in streams.items():
        publishers[name].publish(message)
    rospy.loginfo('Imported 923: %d points, %d geometric components, frame=%s',
                  len(points), len(graph['nodes'])-1, FRAME)
    rospy.spin()


if __name__ == '__main__':
    main()
