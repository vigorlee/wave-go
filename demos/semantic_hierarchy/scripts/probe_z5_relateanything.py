#!/usr/bin/env python3
"""Exploratory image-relation probe; never writes relations into the PCD map."""
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw
from relsgg import RelateAnything


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'outputs/imported_12_20260926/z5_features'
SOURCE = DATA / '09_left_stairs_detail.png'
NAMES = ['下段楼梯', '楼梯平台', '上段楼梯', '垂直墙']
# Image-space regions are traced on the final RViz render.  They are not
# camera detections or projectable annotations in the source PCD frame.
BOXES = np.asarray([[905, 322, 1210, 520],
                    [875, 312, 922, 394],
                    [600, 194, 887, 384],
                    [0, 220, 500, 700]], dtype=np.float32)
VOCABULARY = ['left of', 'right of', 'above', 'below', 'beside',
              'on top of', 'connected to', 'part of', 'in front of', 'behind']


def main():
    image = Image.open(SOURCE).convert('RGB').crop((22, 196, 1828, 897))
    image_path = DATA / 'relateanything_input.png'
    image.save(image_path)
    drawn = image.copy()
    canvas = ImageDraw.Draw(drawn)
    for i, box in enumerate(BOXES):
        canvas.rectangle(tuple(map(int, box)), outline=(250, 223, 89), width=3)
        canvas.text((int(box[0])+4, int(box[1])+4), str(i), fill='white')
    drawn.save(DATA / 'relateanything_regions.png')
    model = RelateAnything.from_pretrained('maelic/relsgg-vits16plus', device='cuda')
    model.set_vocabulary(VOCABULARY)
    predictions = model.predict(image, BOXES, topk=20)
    repo = ROOT / '.runtime/RelateAnything'
    commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                                     text=True).strip()
    report = {'model': 'maelic/relsgg-vits16plus', 'code_commit': commit,
              'input': str(image_path), 'input_kind': 'cropped RViz point-cloud render',
              'not_registered_camera_rgb': True,
              'box_names': NAMES, 'boxes_xyxy': BOXES.tolist(),
              'vocabulary': VOCABULARY,
              'predictions': [repr(p) for p in predictions],
              'pointcloud_labels_modified': False,
              'reason_not_integrated': 'Input is a rendered scan, not a registered RGB image; relations are low confidence and geometrically inconsistent.'}
    (DATA / 'relateanything_probe.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
