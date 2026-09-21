#!/usr/bin/env python3
"""Create Chinese thesis figure copies with centered captions below each image.

The map pixels are unchanged. Caption styling follows the supplied BIT rule:
centered Chinese serif text, single line, with whitespace before and after.
"""
import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1];DATA=ROOT/'outputs/fresh_semantic_20260919';SRC=DATA/'outputs/fastlio_comparison';OUT=DATA/'thesis_figures_cn';OUT.mkdir(parents=True,exist_ok=True)
SERIF='/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc';BOLD='/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc'

def add_caption(src,dst,caption):
    im=Image.open(src).convert('RGB');w,h=im.size;band=112
    canvas=Image.new('RGB',(w,h+band),'white');canvas.paste(im,(0,0));d=ImageDraw.Draw(canvas);font=ImageFont.truetype(SERIF,25);bold=ImageFont.truetype(BOLD,25)
    # Caption is intentionally outside the plotted pixels, matching Word's
    # below-figure placement; the same text is recorded in the manifest.
    box=d.textbbox((0,0),caption,font=font);d.text(((w-(box[2]-box[0]))/2,h+30),caption,font=font,fill=(32,42,56))
    canvas.save(dst,dpi=(300,300))

def main():
    entries=[]
    for key,title in [('overview','三种表示的立体总览'),('side','三种表示的跨层侧视'),('top','三种表示的俯视')]:
        src=SRC/('fresh_'+key+'_comparison_cn.png');dst=OUT/f'图_新校正地图_{key}.png';caption=f'图 X 新校正地图的{title}（测量点云、语义目标框与场景图）';add_caption(src,dst,caption);entries.append(dict(file=str(dst),source=str(src),caption=caption))
    src=DATA/'outputs/rgbd_models/panels_cn/frame_0080.png';dst=OUT/'图_RGBD三模型语义结果.png';caption='图 X 新采集 RGB-D 图像的相对深度、目标检测与语义掩膜结果';add_caption(src,dst,caption);entries.append(dict(file=str(dst),source=str(src),caption=caption))
    for key,title in [('06_hovsg_upstream_floors','原版楼层分割结果'),('07_utonia_features','Utonia特征颜色可视化')]:
        src=SRC/'fastlio_cn'/(key+'_map.png');dst=OUT/f'图_{title}.png';caption=f'图 X {title}（与校正地图使用同一坐标）';add_caption(src,dst,caption);entries.append(dict(file=str(dst),source=str(src),caption=caption))
    manifest=dict(status='passed',language='中文',caption_font='Noto Serif CJK SC（宋体风格预览；排入论文时设置宋体五号）',caption_position='图下方居中',line_spacing='单倍（图像预览中留白；Word排版时设置段前段后6磅）',figure_numbering='当前使用“图”占位；排入正文后按章节重新编号',dpi=300,figures=entries)
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(json.dumps(manifest,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
