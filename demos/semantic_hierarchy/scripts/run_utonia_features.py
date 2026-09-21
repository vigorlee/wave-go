#!/usr/bin/env python3
"""Official Utonia encoder on measured OfficeWorld XYZ, without RGB/normals.

This is feature inference, NOT a semantic class predictor or new SLAM run.
Coarse feature colors are mapped back to the original measured points by their
input voxel membership; no new points or class labels are manufactured.
"""
import argparse
import ast
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch
from scene_profile import DATA_ROOT

ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT/'.runtime/Utonia'
OUT=DATA_ROOT/'outputs/academic_upgrade'
sys.path.insert(0,str(UPSTREAM))
import utonia


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--voxel',type=float,default=.12)
    ap.add_argument('--patch-size',type=int,default=256)
    args=ap.parse_args()
    OUT.mkdir(exist_ok=True,parents=True)
    checkpoint=ROOT/'.runtime/utonia_weights/utonia.pth'
    expected_sha='a0a2e5e234ce943082ae9ba26bcff74e285f16ed8d04fed5c36ffe8dc30ea353'
    actual_sha=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    assert actual_sha==expected_sha, 'Checkpoint SHA mismatch'
    commit=subprocess.check_output(['git','-C',str(UPSTREAM),'rev-parse','HEAD'],text=True).strip()
    assert commit=='da776a0bd3a48c6df83ac2ae0e27b26141cc7e31'
    utonia.utils.set_seed(37)
    xyz=np.load(DATA_ROOT/'outputs/reference_demo/mapping_replay.npz')['points']
    keys=np.floor(xyz/args.voxel).astype(np.int32)
    _,indices,inverse=np.unique(keys,axis=0,return_index=True,return_inverse=True)
    sample=xyz[indices].copy()
    print('INPUT',len(xyz),'ENCODER_POINTS',len(sample),'VOXEL',args.voxel,flush=True)
    model=utonia.load(str(checkpoint),custom_config=dict(enable_flash=False,enc_patch_size=[args.patch_size]*5)).cuda().eval()
    point=dict(coord=sample,color=np.zeros_like(sample),normal=np.zeros_like(sample))
    transform=utonia.transform.default(scale=.5)
    point=transform(point)
    print('TRANSFORMED',len(point['coord']),flush=True)
    # Execute the unchanged PCA coloring helper from the official demo.
    source=UPSTREAM/'demo/0_pca_indoor.py'
    tree=ast.parse(source.read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='get_pca_color')
    scope={'torch':torch}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),scope)
    start=time.perf_counter();torch.cuda.reset_peak_memory_stats()
    with torch.inference_mode():
        for k,v in point.items():
            if isinstance(v,torch.Tensor):point[k]=v.cuda(non_blocking=True)
        point=model(point)
        # Exactly the official demo's two-level feature concatenation.
        for _ in range(2):
            parent=point.pop('pooling_parent');pool_inverse=point.pop('pooling_inverse')
            parent.feat=torch.cat([parent.feat,point.feat[pool_inverse]],dim=-1)
            point=parent
        while 'pooling_parent' in point:
            parent=point.pop('pooling_parent');pool_inverse=point.pop('pooling_inverse')
            parent.feat=point.feat[pool_inverse];point=parent
        assert torch.isfinite(point.feat).all()
        feature_dim=point.feat.shape[-1]
        print('FEATURES',tuple(point.feat.shape),flush=True)
        rgb=scope['get_pca_color'](point.feat,brightness=1.2,center=True)
        sample_rgb=rgb[point.inverse].cpu().numpy()
        torch.cuda.synchronize()
    elapsed=time.perf_counter()-start
    display_rgb=np.rint(sample_rgb[inverse]*255).clip(0,255).astype(np.uint8)
    assert display_rgb.shape==(len(xyz),3)
    np.savez_compressed(OUT/'utonia_features.npz',colors=display_rgb,
                        encoder_point_indices=indices.astype(np.int32),
                        input_voxel_inverse=inverse.astype(np.int32),
                        encoder_rgb=np.rint(sample_rgb*255).clip(0,255).astype(np.uint8))
    meta=dict(status='completed',repository='https://github.com/Pointcept/Utonia',commit=commit,
              model_repository='Pointcept/Utonia',model_revision='fb39b40d33d215d29d1d4717ffb86287f4700c4d',
              checkpoint_sha256=actual_sha,torch_version=torch.__version__,gpu=torch.cuda.get_device_name(),
              total_measured_points=len(xyz),encoder_input_points=len(sample),input_voxel_m=args.voxel,
              normal_input='absent (zero)',rgb_input='absent (zero)',semantic_labels_input='none',
              transform_scale=.5,attention='official non-Flash path',patch_size=args.patch_size,
              feature_dim=feature_dim,pca='unmodified official get_pca_color helper; seed 37; brightness 1.2',
              stage_seconds=elapsed,peak_gpu_allocated_bytes=torch.cuda.max_memory_allocated(),
              coordinate_changes='none in displayed/exported map; voxel feature colors assigned to original XYZ',
              limitation='Feature similarity colors are not semantic class labels, confidence, or improved map accuracy.')
    (OUT/'utonia_report.json').write_text(json.dumps(meta,indent=2))
    print(json.dumps(meta,indent=2),flush=True)


if __name__=='__main__':main()
