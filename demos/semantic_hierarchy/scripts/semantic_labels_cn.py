"""Canonical Chinese display names for the recorded semantic graph.

The English ontology and relation keys remain the stable machine-readable
identifiers.  These mappings are presentation-only and are shared by the
scene-graph JSON, RViz markers, and RGB-D model panels.
"""

KIND_ZH = {
    'unknown': '未分类',
    'building': '数据容器',
    'floor': '楼层候选',
    'region': '观测区域',
    'stairs': '楼梯',
    'ramp': '坡道',
    'cylinder': '圆柱障碍物',
    'person': '行人',
    'wall': '墙面',
}

RELATION_ZH = {
    'contains': '包含',
    'contains_region': '包含区域',
    'near_region': '邻近区域',
    'bounded_by_candidate': '候选边界',
    'stairs_candidate': '楼梯连接候选',
}

ONTOLOGY_ZH = ['未分类', '楼梯', '坡道', '圆柱障碍物', '行人', '墙面', '地面']


def node_label_zh(node_id, kind):
    """Return a compact label suitable for a graph marker or figure."""
    if kind == 'building':
        return f'{node_id} 场景数据容器'
    return f'{node_id} {KIND_ZH.get(kind, kind)}'


def translate_graph(graph):
    """Add Chinese display fields while preserving every English field."""
    out = dict(graph)
    out['ontology_zh'] = list(ONTOLOGY_ZH)
    out['nodes'] = []
    for node in graph.get('nodes', []):
        item = dict(node)
        item['kind_zh'] = KIND_ZH.get(node.get('kind', ''), node.get('kind', ''))
        item['label_zh'] = node_label_zh(node.get('id', ''), node.get('kind', ''))
        out['nodes'].append(item)
    out['edges'] = []
    for edge in graph.get('edges', []):
        item = dict(edge)
        item['relation_zh'] = RELATION_ZH.get(edge.get('relation', ''), edge.get('relation', ''))
        out['edges'].append(item)
    out['language'] = 'zh-CN display fields added; English identifiers preserved'
    return out
