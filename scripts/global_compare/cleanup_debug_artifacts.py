#!/usr/bin/python3
"""One-time audited cleanup of superseded OfficeWorld debug outputs.

Only generated files under artifacts/ are eligible. Source PCD fixtures,
original rosbag captures, final movies and successful run manifests remain.
"""
import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ART = ROOT / 'artifacts'
FAILED_RUNS = {
    'global_navfn_final_20261001',
    'global_navfn_take3_20261001',
    'global_navfn_take4_20261001',
    'global_navfn_take5_20261001',
    'global_pedestrian_probe_20261001',
    'global_pedestrian_probe2_20261001',
    'global_smac_test1_20261001',
    'global_smac_test2_20261001',
}


def candidates():
    rows = []
    for path in sorted(ART.rglob('*')):
        if not path.is_file():
            continue
        rel = path.relative_to(ART)
        if rel.parts[0] in FAILED_RUNS:
            reason = 'superseded failed or probe run'
        elif path.suffix.lower() == '.pcd':
            reason = 'previously recorded PCD explicitly requested for deletion'
        elif path.name == 'registered_xyzi.float32':
            reason = 'redundant unbounded debug scan capture'
        else:
            continue
        rows.append({'relative_path': str(rel), 'bytes': path.stat().st_size,
                     'reason': reason})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    args = ap.parse_args()
    report_dir = ART / 'lio_map_review_20261002'
    report_dir.mkdir(exist_ok=True)
    rows = candidates()
    manifest = report_dir / 'deleted_files_manifest_20261002.json'
    if manifest.exists() and json.loads(manifest.read_text()).get('applied'):
        manifest = report_dir / f'deleted_files_manifest_{datetime.now():%Y%m%d_%H%M%S}.json'
    manifest.write_text(json.dumps({'created_at': datetime.now().isoformat(),
                                    'applied': args.apply,
                                    'files': rows,
                                    'total_bytes': sum(r['bytes'] for r in rows)},
                                   ensure_ascii=False, indent=2))
    if args.apply:
        for row in rows:
            path = ART / row['relative_path']
            if not path.resolve().is_relative_to(ART.resolve()) or path.is_symlink():
                raise RuntimeError(f'Unsafe path: {path}')
            path.unlink()
        for name in FAILED_RUNS:
            d = ART / name
            if d.is_dir():
                shutil.rmtree(d)
    print(json.dumps({'applied': args.apply, 'files': len(rows),
                      'bytes': sum(r['bytes'] for r in rows),
                      'GiB': round(sum(r['bytes'] for r in rows) / 2**30, 3),
                      'manifest': str(manifest)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
