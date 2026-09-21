#!/usr/bin/env python3
"""Add Chinese display fields to the recorded semantic graph artifacts.

English ontology keys, class IDs, relations, scores, and coordinates are kept
unchanged.  The generated fields are for RViz/README/figure display only.
"""
import argparse
import json
from pathlib import Path

from semantic_labels_cn import translate_graph, KIND_ZH, RELATION_ZH


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', type=Path)
    args = ap.parse_args()
    root = (args.dataset or Path(__file__).resolve().parents[1] / 'outputs' / 'fresh_semantic_20260919').resolve()
    out = root / 'outputs'
    graph_path = out / 'scene_graph.json'
    graph = json.loads(graph_path.read_text())
    translated = translate_graph(graph)
    graph_path.write_text(json.dumps(translated, ensure_ascii=False, indent=2))
    (out / 'scene_graph_cn.json').write_text(json.dumps(translated, ensure_ascii=False, indent=2))

    annotations_path = out / 'reference_demo' / 'semantic_annotations.json'
    if annotations_path.exists():
        annotations = json.loads(annotations_path.read_text())
        for item in annotations:
            kind = item.get('kind', '')
            item['kind_zh'] = KIND_ZH.get(kind, kind)
            item['label_zh'] = f"{item.get('id', '')} {KIND_ZH.get(kind, kind)}"
        annotations_path.write_text(json.dumps(annotations, ensure_ascii=False, indent=2))
        (out / 'reference_demo' / 'semantic_annotations_cn.json').write_text(
            json.dumps(annotations, ensure_ascii=False, indent=2))

    report_path = out / 'report.json'
    if report_path.exists():
        report = json.loads(report_path.read_text())
        report['display_language'] = 'zh-CN'
        report['translation_only'] = True
        report['relation_translation'] = RELATION_ZH
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({'status': 'passed', 'scene_graph': str(graph_path),
                      'scene_graph_cn': str(out / 'scene_graph_cn.json')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
