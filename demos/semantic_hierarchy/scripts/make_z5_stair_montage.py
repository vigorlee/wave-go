#!/usr/bin/env python3
"""Layout four unaltered frames from one recorded MATRiX stair trial."""
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'outputs/imported_12_20260926/z5_features/mujoco_stair_proxy/live_frames'
STAGES = ['起步', '下段楼梯', '上段楼梯', '上层平台']
FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def main():
    provenance = json.loads((DATA / 'frame_provenance.json').read_text())
    records = provenance['frames']
    if len(records) != 4 or len({item['raw_image'] for item in records}) != 4:
        raise ValueError('Expected four distinct frames from one recorded trial')
    trial = json.loads(Path(provenance['trial_result']).read_text())
    if not trial['controller_success']:
        raise ValueError('Recorded trial did not reach the upper landing')
    tile_w, tile_h = 760, 440
    gap, margin, top = 20, 30, 90
    canvas = Image.new('RGB', (2 * tile_w + 2 * margin + gap,
                               2 * (tile_h + 52) + 2 * margin + gap + top), '#f5f7fa')
    draw = ImageDraw.Draw(canvas)
    title_font = ImageFont.truetype(FONT_PATH, 31)
    label_font = ImageFont.truetype(FONT_PATH, 21)
    footer_font = ImageFont.truetype(FONT_PATH, 17)
    draw.text((margin, 25), 'Go2-W · 实测楼梯剖面代理上的连续运动',
              font=title_font, fill='#19324d')
    for index, (stage, record) in enumerate(zip(STAGES, records)):
        source = Image.open(record['focus_image']).convert('RGB')
        # Uniform resize and layout only; the underlying MuJoCo image is kept.
        thumb = source.resize((tile_w, tile_h), Image.Resampling.LANCZOS)
        left = margin + (index % 2) * (tile_w + gap)
        upper = top + (index // 2) * (tile_h + 52 + gap)
        canvas.paste(thumb, (left, upper))
        draw.rectangle((left, upper, left + tile_w - 1, upper + tile_h - 1),
                       outline='#b8c7d5', width=2)
        label = (f'{index + 1:02d}  {stage}   '
                 f't={record["elapsed_wall_s"]:.1f} s   '
                 f'x={record["x_m"]:.2f} m   z={record["z_m"]:.2f} m')
        draw.text((left + 4, upper + tile_h + 9), label,
                  font=label_font, fill='#19324d')
    draw.text((margin, canvas.height - 26),
              '同一次 MATRiX MuJoCo / DreamWaQ 试跑；t 为录制墙钟时间，x/z 为仿真坐标；局部碰撞代理。',
              font=footer_font, fill='#52667b')
    output = DATA / 'stair_trial_four_stages.png'
    canvas.save(output)
    print(output)


if __name__ == '__main__':
    main()
