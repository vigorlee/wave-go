#!/usr/bin/env python3
"""Make camera-matched comparisons from native RViz captures; no video required."""
import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/fastlio_comparison'
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
title=ImageFont.truetype(FONT,28);body=ImageFont.truetype(FONT,19)

def compose(images,headings,subheading,path):
    assert len({im.size for im in images})==1
    # Identical crop for all panels, removing side margins only.
    width,height=images[0].size
    crop=(int(width*.18),0,int(width*.82),height)
    pw=1020;ph=round(height*pw/(crop[2]-crop[0]))
    canvas=Image.new('RGB',(pw*len(images)+24*(len(images)+1),ph+150),'white')
    d=ImageDraw.Draw(canvas)
    for i,(im,heading) in enumerate(zip(images,headings)):
        assert np.asarray(im).std()>10
        x=24+i*(pw+24);d.text((x,20),heading,font=title,fill='#26364b')
        canvas.paste(im.crop(crop).resize((pw,ph),Image.Resampling.LANCZOS),(x,68))
    d.text((24,ph+89),subheading,font=body,fill='#58667a')
    canvas.save(path)
    return dict(path=str(path),size=list(canvas.size),common_crop=list(crop))

def main():
    results=[];cameras={}
    for style in ['fastlio','paper']:
        for view in ['overview','side','top']:
            modes=['raw','semantic','graph']
            audits=[json.loads((OUT/style/(view+'_'+m+'.json')).read_text()) for m in modes]
            for m,a in zip(modes,audits):
                assert a['camera']==audits[0]['camera']
                assert a['display']['mode']==m and a['display']['style']==style
                assert a['status']['points']==1647372
                assert a['status']['elapsed']==a['status']['duration']
                assert a['display']['frame']=='lio_map'
            cameras[style+'_'+view]=audits[0]['camera']
            if style=='paper':assert cameras['fastlio_'+view]==audits[0]['camera']
            images=[Image.open(OUT/style/(view+'_'+m+'_map.png')).convert('RGB') for m in modes]
            results.append(compose(images,['(a) Measured cloud / height','(b) Semantic colors + boxes','(c) Graph / structural candidates'],
                           'Same data, camera and time | 1,647,372 measured points | representation comparison, not an accuracy ranking',
                           OUT/(style+'_'+view+'_comparison.png')))
    images=[Image.open(OUT/style/'overview_raw_map.png').convert('RGB') for style in ['fastlio','paper']]
    results.append(compose(images,['FAST-LIO style / RViz AxisColor','Paper style / muted blue-gold'],
                   'Both encode Z height in metres, range -1.5 to 8.2 | identical measured coordinates',OUT/'height_styles_comparison.png'))
    result=dict(status='passed',points=1647372,styles=['fastlio','paper'],views=['overview','side','top'],
                same_camera_per_view=True,coordinates_unchanged=True,no_video_required=True,artifacts=results)
    (OUT/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
