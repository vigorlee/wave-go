"""Metric placement of the existing, provenance-tagged scene graph in RViz."""
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from publish_scene import marker, text, lines
from academic_palette import CLASS_COLORS
from scene_profile import CHINESE
from semantic_labels_cn import KIND_ZH

COLORS = dict(CLASS_COLORS, building=(113,86,151),region=(173,130,70))
CN_KIND=KIND_ZH


def graph_markers(scene, visible, dark=False):
    nodes = {n['id']: n for n in scene['nodes'] if n['id'] in visible or n['kind'] == 'building'}
    marks, labels = [Marker(action=Marker.DELETEALL)], [Marker(action=Marker.DELETEALL)]
    if not visible:
        return MarkerArray(markers=marks), MarkerArray(markers=labels)
    for i, n in enumerate(nodes.values()):
        pos = np.array(n['position'])
        kind = n['kind']
        size = {'building': .65, 'floor': .6, 'region': .44, 'wall': .22}.get(kind, .4)
        node = marker('metric_graph_nodes', i, Marker.SPHERE, pos, COLORS[kind])
        node.scale.x = node.scale.y = node.scale.z = size
        marks.append(node)
        if kind != 'wall':
            title = n['id'] + ' ' + (n.get('kind_zh', CN_KIND.get(kind,kind)) if CHINESE else kind)
            if kind in ['floor', 'region']: title += '*'
            labels.append(text('metric_graph_labels', i, title, pos + [0, 0, .5], (225,231,240) if dark else (38,47,66), .37))
    for i, edge in enumerate(scene['edges']):
        if edge['source'] not in nodes or edge['target'] not in nodes:
            continue
        a = nodes[edge['source']]['position']; b = nodes[edge['target']]['position']
        is_stair = edge['relation'] == 'stairs_candidate'
        color = (40,126,225) if is_stair else (107,116,134)
        marks.append(lines('metric_graph_edges', i, [(a,b)], color, .065 if is_stair else .025, .95))
    return MarkerArray(markers=marks), MarkerArray(markers=labels)
