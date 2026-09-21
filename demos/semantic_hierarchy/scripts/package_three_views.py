#!/usr/bin/env python3
"""Compose actual RViz captures and validate representation consistency."""
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/three_representations'


def main():
    names=['01_graph','02_semantic_boxes','03_raw_cloud']
    shots=[json.loads((OUT/(n+'.json')).read_text()) for n in names]
    for shot in shots:
        assert shot['camera']==shots[0]['camera'], 'Comparison camera differs'
        assert shot['status']['points']==1647372
        assert shot['status']['elapsed']==shot['status']['duration']
        assert shot['display']['frame']=='lio_map'
    assert shots[2]['display']['enabled']==['Raw measured cloud']
    assert 'Metric graph' in shots[0]['display']['enabled']
    assert 'Model object outlines' in shots[1]['display']['enabled']
    scene=json.loads((ROOT/'outputs/scene_graph.json').read_text())
    ids={n['id'] for n in scene['nodes']}
    assert len(ids)==len(scene['nodes'])==28
    assert len(scene['edges'])==28
    assert all(e['source'] in ids and e['target'] in ids for e in scene['edges'])
    graph_points=np.array([n['position'] for n in scene['nodes']])
    assert np.isfinite(graph_points).all()
    native=np.load(ROOT/'outputs/reference_demo/model_semantics.npz')
    source=np.load(ROOT/'data/semantic_map.npz')
    assigned=native['source_index']>=0
    assert np.array_equal(native['labels'][assigned],source['labels'][native['source_index'][assigned]])
    assert np.count_nonzero(native['labels']==4)==0
    upstream=json.loads((OUT/'hovsg_floor_probe/result.json').read_text())
    upstream_shot=json.loads((OUT/'06_hovsg_upstream_floors.json').read_text())
    assert upstream['status']=='completed' and upstream['num_floors']==2
    assert sum(upstream['points_per_floor'])+upstream['unassigned_points']==1647372
    assert upstream_shot['display']['enabled']==['HOV-SG upstream floors']
    assert upstream_shot['camera']==shots[0]['camera']
    # A common screen crop removes empty margins only; equal view/scale for all.
    crop=(365,100,1365,815)
    images=[Image.open(OUT/(name+'_map.png')).convert('RGB') for name in names]
    assert {im.size for im in images}=={(1734,820)}
    for im in images:
        assert np.asarray(im).std()>10, 'Blank capture'
    cropped=[im.crop(crop) for im in images]
    canvas=Image.new('RGB',(3088,900),'white');draw=ImageDraw.Draw(canvas)
    font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    title=ImageFont.truetype(font,30);body=ImageFont.truetype(font,20)
    labels=['(a) Graph','(b) Semantic colors + boxes','(c) Measured point cloud']
    subtitles=['28 nodes / 28 edges | structural candidates','DINO + SAM2 labels | 7 object clusters','1,647,372 measured points | 4 cm voxels']
    for i,(im,label,sub) in enumerate(zip(cropped,labels,subtitles)):
        x=22+i*1022
        draw.text((x,20),label,font=title,fill='#23364e')
        canvas.paste(im,(x,73))
        draw.text((x,805),sub,font=body,fill='#45566d')
    draw.text((22,855),'Same OfficeWorld recording, coordinate frame, camera and time. Offline semantic / graph overlay; no accuracy ground truth.',font=body,fill='#58667a')
    canvas.save(OUT/'three_representations.png')
    video=json.loads(subprocess.check_output(['/usr/bin/ffprobe','-v','error','-show_entries','stream=width,height,r_frame_rate','-show_entries','format=duration,size','-of','json',str(OUT/'three_representations_demo.mp4')],text=True))
    assert float(video['format']['duration'])>=65
    assert video['streams'][0]['width']==2040 and video['streams'][0]['height']==1140
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir() if p.suffix in ['.png','.mp4']}
    result=dict(status='passed',camera_identical=True,frames_identical=True,
                all_final_point_counts=1647372,raw_has_no_semantics_or_graph=True,
                graph_nodes=28,graph_edges=28,original_model_class_ids_preserved=True,
                images_same_size=[1734,820],composite_common_crop=list(crop),
                composite_size=[3088,900],video=video,sha256=files,
                hovsg_floor_probe=dict(commit=upstream['commit'],scope=upstream['scope'],
                                      floors=upstream['num_floors'],points=upstream['points_per_floor'],
                                      unassigned=upstream['unassigned_points']),
                limits='Integrity and visualization checks, not semantic or SLAM accuracy evaluation')
    (OUT/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='sha256'},indent=2))


if __name__=='__main__':main()
