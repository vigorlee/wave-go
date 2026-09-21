#!/usr/bin/env python3
"""Compose native RViz captures without altering the rendered map content."""
import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'outputs/level_fix/fresh_20260919_02';src=run/'comparison_views';out=run/'comparisons';out.mkdir(exist_ok=True)
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
f=lambda n:ImageFont.truetype(font,n)
checks=[]
for view,label in [('overview','立体总览'),('side','跨层侧视'),('stairs','楼梯剖面'),('top','俯视')]:
 images=[Image.open(src/f'{k}_{view}.png').convert('RGB') for k in ['old','new']];meta=[json.loads((src/f'{k}_{view}.json').read_text()) for k in ['old','new']]
 for key in ['distance','pitch','yaw','focal']:assert meta[0][key]==meta[1][key]
 assert images[0].size==images[1].size
 ww,hh=images[0].size;canvas=Image.new('RGB',(ww*2+72,hh+224),'#f7f9fc');d=ImageDraw.Draw(canvas)
 d.text((24,12),'OfficeWorld / '+label+' · 新旧地图对比',font=f(34),fill='#1d3045')
 titles=['旧记录 · 原始 PCD 保留','新采集 · 楼层结构校正（已知层高 5.51 m）']
 for i,im in enumerate(images):
  x=24+i*(ww+24);d.text((x,75),titles[i],font=f(25),fill='#33485d');canvas.paste(im,(x,120))
 d.text((24,hh+140),'同一视角 / Z 高度着色   ·   两层法向夹角：3.95° → 0.38°   ·   校正版采用楼层先验，纯 LIO 结果另存',font=f(24),fill='#41556b')
 canvas.save(out/f'{view}_before_after.png');checks.append(dict(view=view,same_camera=True,images=[str(src/f'{k}_{view}.png') for k in ['old','new']]))
(out/'capture_validation.json').write_text(json.dumps(dict(passed=True,comparisons=checks,note='Captured native RViz. Same-camera geometric views, not an independent SLAM benchmark.'),indent=2))
print(out)
