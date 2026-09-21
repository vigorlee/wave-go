#!/usr/bin/env python3
"""Render Chinese, paper-ready copies of the recorded RGB-D model panels.

The metric data, masks, boxes, scores, and class IDs come from the recorded
Grounding DINO + SAM2 result. Only the display language of the accepted model
class is translated; the raw model label stays in each frame JSON.
"""
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/'outputs/fresh_semantic_20260919'
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
COLORS=np.array([[85,100,115],[45,170,255],[180,105,250],[255,165,35],[250,65,135],[155,175,195],[65,195,135],[235,205,65]],np.uint8)
CN=['未分类','楼梯','坡道','圆柱障碍物','行人','墙面','地面','楼梯/坡道待定']

def fit_font(size,bold=False):
    return ImageFont.truetype(BOLD if bold else FONT,size)

def main():
    src=DATA/'outputs/rgbd_models';dst=src/'panels_cn';dst.mkdir(parents=True,exist_ok=True)
    rgbdir=DATA/'data/rgbd';frames=json.loads((rgbdir/'frames.json').read_text());report=json.loads((DATA/'outputs/reference_demo/replay_report.json').read_text())
    selected=sorted(src.glob('panels/frame_*.png'))
    for panel in selected:
        idx=int(panel.stem.split('_')[-1]);meta=json.loads((src/'frames'/f'{idx:04d}.json').read_text());cache=np.load(src/'frames'/f'{idx:04d}.npz');
        rgb=np.array(Image.open(rgbdir/frames[idx]['rgb']).convert('RGB'));sem=cache['semantic_labels'];depth=cache['relative_depth'].astype(np.float32);objects=meta['objects'];h,w=rgb.shape[:2]
        overlay=(rgb*.6+COLORS[sem]*.4).astype(np.uint8);overlay[sem==0]=rgb[sem==0]
        pil=Image.fromarray(overlay);draw=ImageDraw.Draw(pil);font=fit_font(18)
        for o in objects:
            x1,y1,x2,y2=map(int,o['box_xyxy']);c=tuple(map(int,COLORS[o['class_id']]))
            draw.rectangle((x1,y1,x2,y2),outline=c,width=3)
            # The box comes from the accepted Grounding DINO + SAM2 result.
            # Only its display language changes; the raw model text remains in metadata.
            label=f'{o.get("label_zh",CN[int(o["class_id"])])} {o["detection_score"]:.2f}'
            tw=draw.textbbox((0,0),label,font=font);draw.rectangle((x1,max(0,y1-28),x1+tw[2]+8,max(1,y1)),fill=(12,20,32));draw.text((x1+4,max(0,y1-26)),label,font=font,fill=c)
        overlay=np.array(pil)
        lo,hi=np.percentile(depth,[2,98]);vis=cv2.applyColorMap((np.clip((depth-lo)/max(1e-6,hi-lo),0,1)*255).astype(np.uint8),cv2.COLORMAP_INFERNO)[:,:,::-1]
        tiles=[]
        titles=['录制RGB图像','相对深度估计（Depth Anything V2）','目标检测与掩膜（Grounding DINO + SAM2）','语义类别掩膜']
        imgs=[rgb,vis,overlay,COLORS[sem]]
        for title,img in zip(titles,imgs):
            tile=np.zeros((310,480,3),np.uint8);tile[:40]=[12,20,32];tile[40:]=cv2.resize(img,(480,270));tp=Image.fromarray(tile);td=ImageDraw.Draw(tp);td.text((12,8),title,font=fit_font(18,True),fill=(230,240,250));tiles.append(np.array(tp))
        canvas=np.vstack([np.hstack(tiles[:2]),np.hstack(tiles[2:])]);footer=Image.new('RGB',(960,62),(12,20,32));fd=ImageDraw.Draw(footer);stamp=frames[idx]['stamp']-report['first_stamp'];fd.text((14,13),f'预训练模型 · 记录时间 {stamp:.1f} s · {len(objects)} 个候选目标 · 深度图为相对值',font=fit_font(18),fill=(100,235,190));out=np.vstack([canvas,np.array(footer)])
        Image.fromarray(out).save(dst/panel.name)
    (dst/'README.txt').write_text('中文论文图副本：标题、目标框标签、页脚已汉化；数据和模型输出与 ../panels 相同。\n',encoding='utf-8')
    print('written',len(selected),'Chinese model panels')

if __name__=='__main__':main()
