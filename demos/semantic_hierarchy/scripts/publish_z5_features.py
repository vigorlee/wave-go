#!/usr/bin/env python3
"""Publish two independent Z<=5 m feature maps for native RViz display."""
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray

from imported_923_io import read_pcd
from publish_scene import cloud_msg, marker, lines, text, box_edges
from publish_roofless_import import xyz_and_rgb

COLORS = {'ground': (106, 176, 145), 'stairs': (74, 142, 222),
          'ramp': (177, 143, 212), 'other_obstacle': (224, 153, 100),
          'vertical_wall': (224, 119, 124), 'stair_landing': (230, 190, 97),
          'upper_floor': (206, 211, 126), 'upper_corridor': (206, 211, 126)}


def wall_edges(evidence, thickness=.18):
    slope, intercept = evidence['slope_x_per_y'], evidence['x_intercept_m']
    ylo, yhi = evidence['y_range_m']
    zlo, zhi = evidence['z_range_m']
    normal = np.asarray([1., -slope]) / np.sqrt(1 + slope*slope)
    corners = []
    for i in range(8):
        y = yhi if i & 1 else ylo
        xy = (np.asarray([slope*y + intercept, y]) +
              (thickness / 2 if i & 2 else -thickness / 2) * normal)
        corners.append(np.r_[xy, zhi if i & 4 else zlo])
    return [(corners[i], corners[i ^ (1 << bit)])
            for i in range(8) for bit in range(3) if not i & (1 << bit)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, type=Path)
    args = parser.parse_args()
    data = args.data
    graph = json.loads((data / 'feature_graph.json').read_text())
    frame = graph['frame']
    rospy.init_node('wavego_' + frame + '_publisher')
    raw = read_pcd(data / 'display_z5_rgb.pcd')
    colored = read_pcd(data / 'display_z5_semantic.pcd')
    labels = np.load(data / 'class_ids.npy')
    xyz, rgb = xyz_and_rgb(raw)
    colored_xyz, semantic_rgb = xyz_and_rgb(colored)
    if (len(labels) != len(xyz) or graph['point_count'] != len(xyz) or
            not np.array_equal(xyz, colored_xyz) or float(xyz[:, 2].max()) > 5.0):
        raise ValueError('Point count, XYZ or Z cutoff mismatch')

    def cloud(points, colors):
        message = cloud_msg(points, colors)
        message.header.frame_id = frame
        message.header.stamp = rospy.Time.now()
        return message

    selected = np.isin(labels, [2, 3, 6, 7, 8, 9, 10])
    streams = {'rgb': cloud(xyz, rgb), 'height': cloud(xyz, semantic_rgb),
               'components': cloud(xyz[selected], semantic_rgb[selected]),
               'context': cloud(xyz[~selected], semantic_rgb[~selected])}
    boxes, graph_shapes, box_labels, graph_labels = ([Marker(action=Marker.DELETEALL)]
                                                      for _ in range(4))
    nodes = graph['nodes']
    positions = {}
    for index, node in enumerate(nodes):
        kind = node['kind']
        color = ((66, 191, 219) if kind == 'stairs' and node.get('level') == 'upper'
                 else COLORS[kind])
        if kind == 'ground':
            position = np.asarray(node['position'])
        else:
            lo, hi = np.asarray(node['bounds'], float)
            position = (lo + hi) / 2
            edges = (wall_edges(node['evidence']) if kind == 'vertical_wall'
                     else [segment for section in node['evidence']['sections']
                           for segment in box_edges(*np.asarray(section['bounds'], float))]
                     if kind == 'upper_corridor' else box_edges(lo, hi))
            boxes.append(lines('feature_boxes', index, edges, color,
                               .065 if kind == 'vertical_wall' else .035, .92))
            label_pos = position.copy()
            label_pos[2] = hi[2] + (.38 if kind == 'vertical_wall' else .35)
            box_labels.append(text('feature_box_labels', index,
                                   f'{node["id"]} {node["label_zh"]}', label_pos,
                                   color, .45))
        positions[node['id']] = position
        shape = marker('feature_graph_nodes', index, Marker.SPHERE, position, color)
        shape.scale.x = shape.scale.y = shape.scale.z = .45 if kind != 'ground' else .7
        graph_shapes.append(shape)
        graph_labels.append(text('feature_graph_labels', index,
                                 f'{node["id"]} {node["label_zh"]}',
                                 position + [0, 0, .52], color, .44))
    stair_relations = {'measured_stair_adjacency', 'registered_measured_stair_adjacency',
                       'measured_upper_floor_adjacency', 'measured_corridor_overlap'}
    stair_links = {frozenset((edge['source'], edge['target']))
                   for edge in graph['edges']
                   if edge['relation'] in stair_relations}
    for index, edge in enumerate(graph['edges']):
        if (edge['relation'] == 'spatial_proximity' and
                frozenset((edge['source'], edge['target'])) in stair_links):
            continue
        a, b = positions[edge['source']], positions[edge['target']]
        measured_stair = edge['relation'] in stair_relations
        graph_shapes.append(lines('metric_graph_edges', index, [(a, b)],
                                  (230, 190, 97) if measured_stair else (123, 171, 205),
                                  .09 if measured_stair else .035,
                                  .98 if measured_stair else .82))

    def marker_array(items):
        for item in items:
            item.header.frame_id = frame
            item.header.stamp = rospy.Time.now()
        return MarkerArray(markers=items)

    streams.update(boxes=marker_array(boxes), graph=marker_array(graph_shapes),
                   box_labels=marker_array(box_labels), graph_labels=marker_array(graph_labels))
    # Focused stair views keep the same measured points and graph frame while
    # hiding unrelated wall/obstacle markers that obscure the tread profiles.
    node_by_id = {node['id']: node for node in nodes}
    for landing in (n for n in nodes if n['kind'] == 'stair_landing'):
        side = 'left' if np.mean(landing['bounds'], axis=0)[1] > 0 else 'right'
        pair_ids = {landing['id'], landing['evidence']['lower_stair_id'],
                    landing['evidence']['upper_stair_id']}
        pair_ids.update(n['id'] for n in nodes if n['kind'] == 'upper_floor' and
                        n['evidence']['adjacent_upper_stair_id'] in pair_ids)
        pair_bounds = np.asarray([node_by_id[node_id]['bounds'] for node_id in pair_ids], float)
        lo = pair_bounds[:, 0].min(axis=0) - [.6, .55, .30]
        hi = pair_bounds[:, 1].max(axis=0) + [.6, .55, .45]
        roi = np.all((xyz >= lo) & (xyz <= hi), axis=1)
        features = roi & np.isin(labels, [2, 8, 9, 10])
        background = roi & np.isin(labels, [0, 1, 4, 5])
        keep_nodes = {i for i, node in enumerate(nodes) if node['id'] in pair_ids}
        keep_edges = {i for i, edge in enumerate(graph['edges'])
                      if edge['source'] in pair_ids and edge['target'] in pair_ids}
        focused_boxes = [item for item in boxes
                         if item.action == Marker.DELETEALL or item.id in keep_nodes]
        focused_graph = [item for item in graph_shapes
                         if item.action == Marker.DELETEALL or
                         (item.ns == 'feature_graph_nodes' and item.id in keep_nodes) or
                         (item.ns == 'metric_graph_edges' and item.id in keep_edges)]
        streams.update({f'{side}_rgb': cloud(xyz[roi], rgb[roi]),
                        f'{side}_height': cloud(xyz[roi], semantic_rgb[roi]),
                        f'{side}_components': cloud(xyz[features], semantic_rgb[features]),
                        f'{side}_context': cloud(xyz[background], semantic_rgb[background]),
                        f'{side}_boxes': marker_array(focused_boxes),
                        f'{side}_graph': marker_array(focused_graph)})
    corridor_nodes = [n for n in nodes if n['kind'] == 'upper_corridor']
    if corridor_nodes:
        corridor = corridor_nodes[0]
        evidence = corridor['evidence']
        wall = next(n for n in nodes if n['kind'] == 'vertical_wall')['evidence']
        wall_x = wall['slope_x_per_y'] * xyz[:, 1] + wall['x_intercept_m']
        relative_x = xyz[:, 0] - wall_x
        ylo, yhi = np.asarray(corridor['bounds'])[:, 1]
        roi = ((xyz[:, 1] > ylo - .5) & (xyz[:, 1] < yhi + .5) &
               (relative_x > -3.0) & (relative_x < .45) &
               (xyz[:, 2] > .5) & (xyz[:, 2] < 2.8))
        features = roi & np.isin(labels, [8, 10])
        background = roi & np.isin(labels, [0, 4, 5])
        keep_ids = {corridor['id']}
        keep_ids.update(n['id'] for n in nodes if n['kind'] == 'upper_floor')
        keep_ids.update(n['evidence']['adjacent_upper_stair_id'] for n in nodes
                        if n['kind'] == 'upper_floor')
        keep_nodes = {i for i, n in enumerate(nodes) if n['id'] in keep_ids}
        keep_edges = {i for i, edge in enumerate(graph['edges'])
                      if edge['source'] in keep_ids and edge['target'] in keep_ids}
        focused_boxes = [item for item in boxes
                         if item.action == Marker.DELETEALL or item.id in keep_nodes]
        focused_graph = [item for item in graph_shapes
                         if item.action == Marker.DELETEALL or
                         (item.ns == 'feature_graph_nodes' and item.id in keep_nodes) or
                         (item.ns == 'metric_graph_edges' and item.id in keep_edges)]
        streams.update({'corridor_rgb': cloud(xyz[roi], rgb[roi]),
                        'corridor_height': cloud(xyz[roi], semantic_rgb[roi]),
                        'corridor_components': cloud(xyz[features], semantic_rgb[features]),
                        'corridor_context': cloud(xyz[background], semantic_rgb[background]),
                        'corridor_boxes': marker_array(focused_boxes),
                        'corridor_graph': marker_array(focused_graph)})
    prefix = '/' + frame
    publishers = {name: rospy.Publisher(prefix + '/' + name, type(message),
                                       queue_size=1, latch=True)
                  for name, message in streams.items()}
    for name, message in streams.items():
        publishers[name].publish(message)
    rospy.loginfo('%s: %d cropped points, %d geometry-backed feature nodes',
                  frame, len(xyz), len(nodes) - 1)
    rospy.spin()


if __name__ == '__main__':
    main()
