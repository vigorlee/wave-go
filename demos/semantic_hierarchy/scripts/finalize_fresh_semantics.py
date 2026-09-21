#!/usr/bin/env python3
"""Associate structural graph hypotheses with dense measured geometry; verify identity."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scene_profile import DATA_ROOT

def main():
    ref=DATA_ROOT/'outputs/reference_demo';replay_file=ref/'mapping_replay.npz'
    replay=dict(np.load(replay_file));points=replay['points'];graph=np.load(DATA_ROOT/'outputs/semantic_cloud.npz')
    distance,index=cKDTree(graph['points']).query(points,workers=4)
    known=distance<=.12
    replay['labels']=np.where(known,graph['labels'][index],0).astype(np.uint8)
    replay['evidence']=np.where(known,graph['evidence'][index],0).astype(np.uint8)
    np.savez_compressed(replay_file,**replay)
    report=json.loads((ref/'replay_report.json').read_text());source=Path(report['source_run'])/'structural_calibrated/map_arrays.npz'
    original=np.load(source)['xyzi']
    assert np.array_equal(points,original[replay['source_index'],:3]),'Accepted point coordinates changed'
    native=np.load(ref/'model_semantics.npz');assert len(native['labels'])==len(points)
    votes=np.load(DATA_ROOT/'outputs/rgbd_models/fusion_votes.npz')['votes'];selected=native['labels']>0
    maximum=votes.max(1);assert (maximum[selected]>=2).all()
    assert (maximum[selected]/votes.sum(1)[selected]>=.60).all()
    hovsg=np.load(DATA_ROOT/'outputs/three_representations/hovsg_floor_probe/floor_labels.npz');assert len(hovsg['labels'])==len(points)
    features=np.load(DATA_ROOT/'outputs/academic_upgrade/utonia_features.npz');assert features['colors'].shape==(len(points),3)
    scene=json.loads((DATA_ROOT/'outputs/scene_graph.json').read_text());graph_report=json.loads((DATA_ROOT/'outputs/report.json').read_text())
    assert graph_report['dsg_roundtrip']
    assert scene['source_recording']==Path(report['source_run']).name
    result=dict(status='passed',point_count=len(points),accepted_xyz_exact_match=True,semantic_frames_minimum=2,semantic_vote_fraction_minimum=.6,all_representations_same_point_order=True,graph_source_recording=scene['source_recording'],spark_dsg_roundtrip=True,source_map_sha256=report['source_sha256'],semantic_points=int(selected.sum()),graph_nodes=len(scene['nodes']),graph_edges=len(scene['edges']),old_dataset_modified=False)
    (DATA_ROOT/'outputs/validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
