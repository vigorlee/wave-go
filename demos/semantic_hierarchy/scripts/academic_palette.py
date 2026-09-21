"""One display palette for clouds, cuboids, graph nodes and legends.

No inference, relabeling or coordinate changes occur here.
"""
import json
from pathlib import Path
import numpy as np
from scene_profile import PROFILE

CONFIG=json.loads((Path(__file__).resolve().parents[1]/'config/academic_palette.json').read_text())
SEMANTIC=np.asarray(CONFIG['semantic_rgb'],dtype=np.uint8)
CLASS_COLORS={name:tuple(map(int,color)) for name,color in zip(CONFIG['class_ids'],SEMANTIC)}
HEIGHT_RANGE=PROFILE.get('height_range_m',CONFIG['height_range_m'])
HEIGHT_LUT=np.asarray(CONFIG['height_lut'],dtype=np.uint8)
PAPER_LUT=np.array([np.interp(np.arange(256),[0,128,255],c) for c in
                    [(40,103,215),(73,139,179),(102,155,116)]],dtype=np.uint8).T

def paper_height_colors(z):
    lo,hi=HEIGHT_RANGE
    indices=np.rint(np.clip((np.asarray(z)-lo)/(hi-lo),0,1)*255).astype(np.int64)
    return PAPER_LUT[indices]


def height_colors(z):
    lo,hi=HEIGHT_RANGE
    indices=np.rint(np.clip((np.asarray(z)-lo)/(hi-lo),0,1)*255).astype(np.int64)
    return HEIGHT_LUT[indices]


def hex_color(rgb):
    return '#'+''.join(f'{int(v):02x}' for v in rgb)
