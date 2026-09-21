#!/usr/bin/env python3
"""Compose native RViz screenshots without modifying the measured scene."""
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/academic_upgrade'

def main():
    names=['01_graph','02_semantic_boxes','03_raw_cloud','07_utonia_features']
    shots=[json.loads((OUT/(n+'.json')).read_text()) for n in names]
    for s in shots:
        assert s['camera']==shots[0]['camera']
        assert s['status']['points']==1647372
        assert s['status']['elapsed']==s['status']['duration']
        assert s['display']['frame']=='lio_map'
    assert shots[2]['display']['height']
    assert shots[2]['display']['enabled']==['Raw measured cloud']
    assert shots[3]['display']['enabled']==['Utonia feature cloud']
    crop=(365,100,1365,815)
    images=[Image.open(OUT/(n+'_map.png')).convert('RGB') for n in names]
    assert all(i.size==(1734,820) for i in images)
    assert all(np.asarray(i).std()>10 for i in images)
    font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    title=ImageFont.truetype(font,30);body=ImageFont.truetype(font,19)
    canvas=Image.new('RGB',(3088,900),'white');d=ImageDraw.Draw(canvas)
    headings=['(a) Graph','(b) Semantic colors + boxes','(c) Geometry / Batlow height']
    subtitles=['28 nodes / 28 edges | structural candidates','DINO + SAM2 classes | light blue = unassigned','1,647,372 measured points | height in metres']
    for i in range(3):
        x=22+i*1022;d.text((x,20),headings[i],font=title,fill='#26364b')
        canvas.paste(images[i].crop(crop),(x,73));d.text((x,805),subtitles[i],font=body,fill='#45566d')
    d.text((22,855),'Same recording, original coordinates and camera. Colors change the representation; map accuracy has not been evaluated.',font=body,fill='#58667a')
    canvas.save(OUT/'academic_three_representations.png')
    # Equal camera/scale and a common crop for a direct old/new geometry view.
    before=Image.open(ROOT/'outputs/three_representations/03_raw_cloud_map.png').convert('RGB')
    old_audit=json.loads((ROOT/'outputs/three_representations/03_raw_cloud.json').read_text())
    assert old_audit['camera']==shots[2]['camera']
    pair=Image.new('RGB',(2066,885),'white');d=ImageDraw.Draw(pair)
    for i,(im,title_text) in enumerate([(before,'Previous / uniform gray'),(images[2],'Updated / Batlow height')]):
        x=22+i*1022;d.text((x,20),title_text,font=title,fill='#26364b');pair.paste(im.crop(crop),(x,73))
    d.text((22,830),'Identical measured XYZ and view. Height range: -1.5 to 8.2 m. No point synthesis or geometric correction.',font=body,fill='#58667a')
    pair.save(OUT/'before_after_height.png')
    video=json.loads(subprocess.check_output(['/usr/bin/ffprobe','-v','error','-show_entries','stream=width,height,r_frame_rate','-show_entries','format=duration,size','-of','json',str(OUT/'three_representations_demo.mp4')],text=True))
    assert 99<float(video['format']['duration'])<110
    assert video['streams'][0]['width']==2040 and video['streams'][0]['height']==1140
    result=dict(status='passed',same_camera=True,all_views_points=1647372,
                height_colormap='batlow',utonia_is_feature_view=True,
                image_canvas=[1734,820],common_crop=list(crop),video=video,
                sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir() if p.suffix in ['.png','.mp4']})
    (OUT/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='sha256'},indent=2))

if __name__=='__main__':main()
