#!/usr/bin/env python3
"""Package camera-matched native RViz screenshots, without modifying map data."""
import json
from PIL import Image
from scene_profile import DATA_ROOT
from PIL import ImageDraw,ImageFont
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';BOLD='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
def compose_cn(images,headings,subheading,path):
    assert len({im.size for im in images})==1
    width,height=images[0].size;crop=(int(width*.18),0,int(width*.82),height);pw=1020;ph=round(height*pw/(crop[2]-crop[0]))
    canvas=Image.new('RGB',(pw*len(images)+24*(len(images)+1),ph+150),'white');d=ImageDraw.Draw(canvas);title=ImageFont.truetype(BOLD,28);body=ImageFont.truetype(FONT,19)
    for i,(im,heading) in enumerate(zip(images,headings)):
        x=24+i*(pw+24);d.text((x,20),heading,font=title,fill='#26364b');canvas.paste(im.crop(crop).resize((pw,ph),Image.Resampling.LANCZOS),(x,68))
    d.text((24,ph+89),subheading,font=body,fill='#58667a');canvas.save(path);return dict(path=str(path),size=list(canvas.size),common_crop=list(crop))

def main():
    out=DATA_ROOT/'outputs/fastlio_comparison';profile=out/'fastlio_cn';count=json.loads((DATA_ROOT/'outputs/reference_demo/replay_report.json').read_text())['unique_measured_points'];results=[]
    for view in ['overview','side','top']:
        modes=['raw','semantic','graph'];audits=[json.loads((profile/f'{view}_{m}.json').read_text()) for m in modes]
        for mode,a in zip(modes,audits):
            assert a['camera']==audits[0]['camera']
            assert a['display']['mode']==mode and a['display']['frame']=='lio_map'
            assert a['status']['points']==count and a['status']['elapsed']==a['status']['duration']
        images=[Image.open(profile/f'{view}_{m}_map.png').convert('RGB') for m in modes]
        results.append(compose_cn(images,['（a）测量点云与高度','（b）语义颜色与目标框','（c）场景图与结构候选'],f'新校正记录｜{count:,} 个测量点｜同一视角与时间｜使用结构层高先验',out/f'fresh_{view}_comparison_cn.png'))
    (out/'comparison_validation_cn.json').write_text(json.dumps(dict(status='passed',points=count,same_camera=True,static_offline_labels=True,language='中文',artifacts=results),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
