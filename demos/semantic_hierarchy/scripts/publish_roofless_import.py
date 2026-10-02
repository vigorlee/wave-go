#!/usr/bin/env python3
"""ROS 1 display topics for an individually processed roofless LAZ map."""
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray

from imported_923_io import read_pcd
from publish_scene import cloud_msg, marker, lines, text, box_edges

PALETTE = [(68, 121, 176), (104, 164, 141), (197, 148, 78),
           (151, 127, 187), (190, 113, 125), (101, 156, 181),
           (201, 166, 103), (116, 143, 183)]


def xyz_and_rgb(raw):
    points = np.stack([raw[k] for k in 'xyz'], axis=1)
    rgb = np.column_stack(((raw['rgb'] >> 16) & 255,
                           (raw['rgb'] >> 8) & 255,
                           raw['rgb'] & 255)).astype(np.uint8)
    return points, rgb


def height_palette(z):
    low, high = np.quantile(z, [.01, .99])
    anchors = np.array([[low, 62, 104, 164],
                        [(low + high) / 2, 76, 151, 157],
                        [high, 207, 180, 112]], dtype=float)
    return np.column_stack([np.interp(z, anchors[:, 0], anchors[:, c])
                            for c in (1, 2, 3)]).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    args = parser.parse_args()
    data = args.data
    graph = json.loads((data / 'geometry_graph.json').read_text())
    frame = graph['frame']
    prefix = '/' + frame
    rospy.init_node('wavego_' + frame + '_publisher')
    original, raw = (read_pcd(data / filename) for filename in
                     ('display_original.pcd', 'display_no_roof.pcd'))
    original_points, original_rgb = xyz_and_rgb(original)
    points, rgb = xyz_and_rgb(raw)
    ids = np.load(data / 'cluster_ids.npy')
    if len(ids) != len(points) or graph['point_count'] != len(points):
        raise ValueError('Graph and display point counts do not match')

    def cloud(xyz, colors):
        message = cloud_msg(xyz, colors)
        message.header.frame_id = frame
        message.header.stamp = rospy.Time.now()
        return message

    selected = ids > 0
    colors = np.asarray(PALETTE, np.uint8)
    clouds = {'before': cloud(original_points, original_rgb),
              'rgb': cloud(points, rgb),
              'height': cloud(points, height_palette(points[:, 2])),
              'components': cloud(points[selected], colors[ids[selected] - 1]),
              'context': cloud(points[~selected],
                               np.full((int((~selected).sum()), 3), (126, 142, 158), np.uint8))}

    boxes, graph_shapes, box_labels, graph_labels = ([Marker(action=Marker.DELETEALL)] for _ in range(4))
    nodes = {node['id']: node for node in graph['nodes']}
    for index, node in enumerate(graph['nodes']):
        pos = np.asarray(node['position'])
        if node['kind'] == 'horizontal_layer':
            color = (170, 203, 203)
            size = .45
            lo, hi = np.asarray(node['bounds'])
            lo[2] = hi[2] = node['z_m']
            graph_shapes.append(lines('horizontal_bounds', index, box_edges(lo, hi), color, .025, .45))
        elif node['kind'] == 'structural_component':
            color = (124, 150, 172)
            size = .42
        else:
            rank = int(node['id'][1:])
            color = PALETTE[(rank - 1) % len(PALETTE)]
            size = .3
            lo, hi = np.asarray(node['bounds'])
            boxes.append(lines('geometry_bounds', index, box_edges(lo, hi), color, .026, .92))
            top = (lo + hi) / 2
            top[2] = hi[2] + .25
            box_labels.append(text('geometry_box_label', index,
                                   f'{node["id"]} {node["label_zh"]}', top, color, .4))
        sphere = marker('geometry_graph_nodes', index, Marker.SPHERE, pos, color)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = size
        graph_shapes.append(sphere)
        graph_labels.append(text('geometry_graph_label', index,
                                 f'{node["id"]} {node["label_zh"]}', pos + [0, 0, .38], color, .38))
    for index, edge in enumerate(graph['edges']):
        a = nodes[edge['source']]['position']
        b = nodes[edge['target']]['position']
        color = ((226, 195, 124) if edge['relation'] == 'height_adjacent' else
                 (115, 157, 181) if edge['relation'] == 'structural_overlap' else
                 (139, 185, 208))
        graph_shapes.append(lines('geometry_graph_edges', index, [(a, b)], color, .06, .9))

    def marker_array(items):
        for item in items:
            item.header.frame_id = frame
            item.header.stamp = rospy.Time.now()
        return MarkerArray(markers=items)

    streams = dict(clouds, boxes=marker_array(boxes), graph=marker_array(graph_shapes),
                   box_labels=marker_array(box_labels), graph_labels=marker_array(graph_labels))
    publishers = {name: rospy.Publisher(prefix + '/' + name, type(message), queue_size=1, latch=True)
                  for name, message in streams.items()}
    for name, message in streams.items():
        publishers[name].publish(message)
    rospy.loginfo('%s: raw display %d, roofless display %d, graph nodes %d',
                  frame, len(original), len(raw), len(graph['nodes']))
    rospy.spin()


if __name__ == '__main__':
    main()
