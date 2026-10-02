#!/usr/bin/python3
"""Review real registered LiDAR returns without writing PCD or raw scan archives.

The saved NPZ is a voxelized copy of an actual LIO run. This tool only changes
presentation; it does not re-register scans or claim to improve odometry.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dense_capture import points_message, trajectory_graph, spectral_rgb
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2w_semantic_map_core import COLORS, LABELS, cluster_obstacles


def load_trajectory(path):
    if not path or not path.exists():
        return np.empty((0, 3), np.float32)
    poses = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        xyz = np.array([row['x'], row['y'], row['z']], np.float32)
        poses.append(xyz)
    if not poses:
        return np.empty((0, 3), np.float32)
    nodes = [poses[0]]
    for xyz in poses[1:]:
        if np.linalg.norm(xyz - nodes[-1]) >= 6.0:
            nodes.append(xyz)
    if np.linalg.norm(poses[-1] - nodes[-1]) >= .25:
        nodes.append(poses[-1])
    return np.asarray(nodes, np.float32).reshape(-1, 3)


def load_semantic_overlay(path, measured):
    """Keep only saved visual labels coincident with this run's LiDAR points."""
    if not path.exists():
        return np.empty((0, 3), np.float32), np.empty(0, np.uint8), {}
    from scipy.spatial import cKDTree
    with np.load(path) as archive:
        xyz = archive['points'].astype(np.float32)
        labels = archive['labels'].astype(np.uint8)
    known = (labels > 0) & (labels < len(LABELS)) & np.isfinite(xyz).all(axis=1)
    xyz, labels = xyz[known], labels[known]
    distance, _ = cKDTree(measured).query(xyz)
    aligned = distance <= .05
    counts = {LABELS[i]: int(np.count_nonzero(labels[aligned] == i))
              for i in np.unique(labels[aligned])}
    counts['rejected_misaligned'] = int(np.count_nonzero(~aligned))
    return xyz[aligned], labels[aligned], counts


def semantic_message(xyz, labels, stamp):
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    data = np.empty(len(xyz), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
                                     ('rgb','<u4'),('label','<u4')])
    for i, field in enumerate(('x','y','z')):
        data[field] = xyz[:, i]
    rgb = COLORS[labels].astype(np.uint32)
    data['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    data['label'] = labels
    fields = [PointField(name=field, offset=4*i,
                         datatype=PointField.FLOAT32 if i < 3 else PointField.UINT32,
                         count=1) for i, field in enumerate(data.dtype.names)]
    return PointCloud2(header=Header(frame_id='lio_map', stamp=stamp), height=1,
                       width=len(data), fields=fields, point_step=20,
                       row_step=20*len(data), data=data.tobytes(), is_dense=True)


def semantic_markers(xyz, labels, stamp):
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker, MarkerArray
    header = Header(frame_id='lio_map', stamp=stamp)
    interesting = np.isin(labels, [1, 2, 3])
    objects = cluster_obstacles(xyz[interesting], labels[interesting],
                                np.ones(np.count_nonzero(interesting), np.float32))
    limits = {1: 1, 2: 1, 3: 1}
    chosen = {label: 0 for label in limits}
    markers = []
    for obj in objects:
        label = obj['class_id']
        if label not in limits or obj['points'] < 20 or chosen[label] >= limits[label]:
            continue
        chosen[label] += 1
        marker = Marker(header=header, ns='saved_visual_semantic_candidates',
                        id=10*label+chosen[label], type=Marker.TEXT_VIEW_FACING,
                        action=Marker.ADD)
        marker.pose.orientation.w = 1.
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, obj['center'])
        object_top = marker.pose.position.z + float(obj['size'][2]) / 2
        marker.pose.position.z = object_top + 1.25
        marker.scale.z = .48
        marker.color.r = marker.color.g = marker.color.b = .98
        marker.color.a = 1.
        marker.text = f"{LABELS[label].upper()}?"
        markers.append(marker)
        leader = Marker(header=header, ns='saved_visual_semantic_leaders',
                        id=10*label+chosen[label], type=Marker.LINE_LIST,
                        action=Marker.ADD)
        leader.pose.orientation.w = 1.
        leader.scale.x = .022
        leader.color.r, leader.color.g, leader.color.b = map(float, COLORS[label] / 255.)
        leader.color.a = .85
        leader.points = [Point(x=marker.pose.position.x, y=marker.pose.position.y,
                               z=object_top),
                         Point(x=marker.pose.position.x, y=marker.pose.position.y,
                               z=marker.pose.position.z-.18)]
        markers.append(leader)
    return MarkerArray(markers=markers)


def load_views(source, alignment):
    with np.load(source) as data:
        xyz = data['points'].astype(np.float32)
    if not np.isfinite(xyz).all():
        raise ValueError('Non-finite registered points')
    transform = np.linalg.inv(np.asarray(json.loads(alignment.read_text())['matrix']))
    world = xyz @ transform[:3, :3].T + transform[:3, 3]
    region = ((world[:, 0] > -8.5) & (world[:, 0] < 11.5) &
              (world[:, 1] > -9) & (world[:, 1] < 7) &
              (world[:, 2] > -1) & (world[:, 2] < 8.5))
    cut = (world[:, 2] < 2.5) | ((world[:, 0] < -1.8) &
                                  (world[:, 1] < 2.2) & (world[:, 2] < 7.2))
    return xyz[region], xyz[region & cut], world[region], world[region & cut]


def render(out, world, cut_world, graph_world):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rng = np.random.default_rng(20261002)
    def sample(a, n):
        return a[rng.choice(len(a), min(n, len(a)), replace=False)]
    full = sample(world, 230000)
    cut = sample(cut_world, 100000)
    fig = plt.figure(figsize=(19.2, 7.2), facecolor='#0d1622')
    gs = fig.add_gridspec(1, 3, left=.035, right=.98, bottom=.105, top=.875, wspace=.13)
    ax = fig.add_subplot(gs[0, 0])
    from matplotlib.colors import LinearSegmentedColormap
    palette = LinearSegmentedColormap.from_list('lio_spectral',
        [(0,'#ff9500'),(.13,'#ffe000'),(.27,'#67eb1a'),(.43,'#00dcbf'),
         (.61,'#0d84eb'),(.78,'#2d33b2'),(1,'#7b2c97')])
    sc = ax.scatter(full[:, 0], full[:, 1], c=full[:, 2], s=.12, alpha=.65,
                    cmap=palette, vmin=0, vmax=6.5, rasterized=True)
    fig.colorbar(sc, ax=ax, label='Height (m)', fraction=.035, pad=.025)
    ax.set(xlim=(-8.5, 11.5), ylim=(-9, 7), xlabel='X (m)', ylabel='Y (m)',
           title='A  Full measured map / elevation')
    ax.set_aspect('equal', adjustable='box')
    ax = fig.add_subplot(gs[0, 1], projection='3d')
    ax.scatter(cut[:, 0], cut[:, 1], cut[:, 2], c=cut[:, 2],
               s=.14, alpha=.54, cmap=palette, vmin=0, vmax=6.5, rasterized=True)
    if len(graph_world):
        node_color = spectral_rgb(graph_world[:, 2], -.4)/255.
        ax.plot(graph_world[:, 0], graph_world[:, 1], graph_world[:, 2]+1.1,
                color='#b9c6d8', linewidth=.65, alpha=.8)
        ax.scatter(graph_world[:, 0], graph_world[:, 1], graph_world[:, 2]+1.1,
                   s=65, c=node_color, edgecolors='#dce8f2', linewidths=.4,
                   depthshade=False)
    ax.set(xlim=(-8, 11), ylim=(-8, 7), zlim=(-.5, 7.5),
           xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)',
           title='B  Architectural cutaway / 3D')
    ax.view_init(elev=32, azim=-65)
    ax = fig.add_subplot(gs[0, 2])
    section = full[np.abs(full[:, 1]) < 2.0]
    ax.scatter(section[:, 0], section[:, 2], c=section[:, 2], s=.19,
               alpha=.6, cmap=palette, vmin=0, vmax=6.5, rasterized=True)
    ax.set(xlim=(-8.5, 11.5), ylim=(-1, 8), xlabel='X (m)', ylabel='Z (m)',
           title='C  Central 4 m corridor / vertical profile')
    for a in fig.axes:
        if a is not fig.axes[1]:
            a.set_facecolor('#111e2b')
            a.tick_params(colors='#c4d4e3')
            a.xaxis.label.set_color('#dce8f2')
            a.yaxis.label.set_color('#dce8f2')
            a.title.set_color('#eaf3fc')
    fig.suptitle('Go2-W OfficeWorld | registered LiDAR / LIO map views | 2026-10-02',
                 color='white', fontsize=17)
    fig.text(.035, .035, 'Measured points only. Spheres/edges are sampled LIO poses; panel B is a cutaway. Height color is not a semantic label.',
             color='#bacddd', fontsize=10)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


def render_semantic(out, world, semantic_xyz, semantic_labels, alignment):
    """Two source-backed plan views; no synthetic points or inferred people."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    transform = np.linalg.inv(np.asarray(json.loads(alignment.read_text())['matrix']))
    semantic_world = semantic_xyz @ transform[:3, :3].T + transform[:3, 3]
    rng = np.random.default_rng(20261002)
    base = world[rng.choice(len(world), min(250000, len(world)), replace=False)]
    fig, axes = plt.subplots(1, 2, figsize=(17.5, 8.1), facecolor='#0c1522')
    palette = LinearSegmentedColormap.from_list('lio_spectral',
        [(0,'#ff9500'),(.13,'#ffe000'),(.27,'#67eb1a'),(.43,'#00dcbf'),
         (.61,'#0d84eb'),(.78,'#2d33b2'),(1,'#7b2c97')])
    axes[0].scatter(base[:,0], base[:,1], c=base[:,2], s=.2, alpha=.8,
                    cmap=palette, vmin=-.4, vmax=6.1, rasterized=True)
    axes[0].set_title('A  Registered LiDAR · height color', color='white', fontsize=16)
    axes[1].scatter(base[:,0], base[:,1], s=.13, c='#71839a', alpha=.15,
                    rasterized=True)
    for label in (6,5,1,2,3):
        select = semantic_labels == label
        if not np.any(select):
            continue
        p = semantic_world[select]
        color = '#%02x%02x%02x' % tuple(COLORS[label])
        axes[1].scatter(p[:,0], p[:,1], s=2.7 if label in (1,2,3) else 1.3,
                        c=color, alpha=.95, label=f'{LABELS[label]}  ·  {len(p):,}',
                        rasterized=True)
    axes[1].set_title('B  Visual semantic candidates · matched to LiDAR',
                      color='white', fontsize=16)
    axes[1].legend(loc='upper right', facecolor='#15263a', edgecolor='#7890a8',
                   labelcolor='white', fontsize=11, markerscale=4)
    for ax in axes:
        ax.set(facecolor='#101d2c', xlim=(-8.5,11.5), ylim=(-9,7),
               xlabel='OfficeWorld X (m)', ylabel='OfficeWorld Y (m)')
        ax.set_aspect('equal')
        ax.tick_params(colors='#d8e7f2')
        ax.xaxis.label.set_color('#d8e7f2')
        ax.yaxis.label.set_color('#d8e7f2')
        ax.grid(color='#42556a', alpha=.25)
    fig.suptitle('Go2-W OfficeWorld | measured 3D map + synchronized visual labels',
                 color='white', fontsize=19)
    fig.text(.05,.03,'Visual candidates: Grounding DINO + SAM2 + metric depth; 5 cm LiDAR alignment gate. '
             'People are intentionally excluded from the static archive. No points were inserted.',
             color='#c8d9e7', fontsize=11)
    fig.tight_layout(rect=(.025,.055,.98,.94), w_pad=3.)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


def publish(full, cut, graph, semantic_xyz, semantic_labels):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    rclpy.init()
    node = Node('wavego_saved_lio_map_review')
    tf_pub = StaticTransformBroadcaster(node)
    origin = TransformStamped()
    origin.header.stamp = node.get_clock().now().to_msg()
    origin.header.frame_id = 'lio_map'
    origin.child_frame_id = 'lio_review_origin'
    origin.transform.rotation.w = 1.0
    tf_pub.sendTransform(origin)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    height = node.create_publisher(type(points_message(full[:1], 0, node.get_clock().now().to_msg())), '/wavego/live_dense/points', qos)
    geometry = node.create_publisher(type(points_message(full[:1], 0, node.get_clock().now().to_msg())), '/wavego/live_dense/geometry', qos)
    cutaway = node.create_publisher(type(points_message(full[:1], 0, node.get_clock().now().to_msg())), '/wavego/live_dense/cutaway', qos)
    from visualization_msgs.msg import MarkerArray
    graph_pub = node.create_publisher(MarkerArray, '/wavego/live_dense/graph', qos)
    semantic_pub = node.create_publisher(type(semantic_message(semantic_xyz[:1], semantic_labels[:1], node.get_clock().now().to_msg())), '/wavego/semantic/map', qos)
    label_pub = node.create_publisher(MarkerArray, '/wavego/semantic/label_markers', qos)
    labels = semantic_markers(semantic_xyz, semantic_labels, node.get_clock().now().to_msg())
    def tick():
        stamp = node.get_clock().now().to_msg()
        height.publish(points_message(full, -.4, stamp))
        geometry.publish(points_message(full, -.4, stamp, 'geometry'))
        cutaway.publish(points_message(cut, -.4, stamp))
        graph_pub.publish(trajectory_graph(graph, stamp, -.4))
        semantic_pub.publish(semantic_message(semantic_xyz, semantic_labels, stamp))
        for marker in labels.markers:
            marker.header.stamp = stamp
        label_pub.publish(labels)
    node.create_timer(4.0, tick)
    tick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--alignment', type=Path, required=True)
    ap.add_argument('--trajectory', type=Path)
    ap.add_argument('--semantic', type=Path)
    ap.add_argument('--image', type=Path)
    ap.add_argument('--semantic-image', type=Path)
    ap.add_argument('--publish', action='store_true')
    args = ap.parse_args()
    full, cut, world, cut_world = load_views(args.source, args.alignment)
    semantic_path = args.semantic or args.source.parent.parent / 'semantic' / 'semantic_map.npz'
    semantic_xyz, semantic_labels, semantic_counts = load_semantic_overlay(semantic_path, full)
    graph = load_trajectory(args.trajectory or args.source.parent / 'lio_trajectory.jsonl')
    transform = np.linalg.inv(np.asarray(json.loads(args.alignment.read_text())['matrix']))
    graph_world = graph @ transform[:3, :3].T + transform[:3, 3]
    print(json.dumps({'source': str(args.source), 'measured_points': len(full),
                      'cutaway_points': len(cut), 'trajectory_nodes': len(graph),
                      'aligned_visual_semantic_points': len(semantic_xyz),
                      'semantic_counts': semantic_counts, 'frame': 'lio_map',
                      'pcd_written': False}), flush=True)
    if args.image:
        render(args.image, world, cut_world, graph_world)
    if args.semantic_image:
        render_semantic(args.semantic_image, world, semantic_xyz, semantic_labels, args.alignment)
    if args.publish:
        publish(full, cut, graph, semantic_xyz, semantic_labels)
