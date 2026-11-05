#!/usr/bin/env python3
"""Measured image text via local Tesseract; bounded asynchronous live worker."""
from __future__ import annotations

import argparse
import csv
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
LOCAL = ROOT / '.local/tesseract'
MIN_CONFIDENCE = 80.0
ROOM_CODE = re.compile(r'^(?:[A-Z]{1,3}[-.]?[0-9]{2,4}|[0-9]{1,4}(?:[.-][0-9]{1,4}){1,2}|[0-9]{2,4})$')
NUMERIC_MIN_CONFIDENCE = 75.0
GENERIC_WORDS = {'door', 'doors', 'floor', 'wall', 'window', 'open', 'close'}
CHARACTERS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789.- '


def tesseract_environment():
    binary = os.environ.get('GO2W_TESSERACT_BIN')
    if not binary:
        local_binary = LOCAL / 'usr/bin/tesseract'
        binary = str(local_binary) if local_binary.is_file() else shutil.which('tesseract')
    env = os.environ.copy()
    env['OMP_THREAD_LIMIT'] = '1'
    if LOCAL.is_dir():
        env['LD_LIBRARY_PATH'] = str(LOCAL / 'usr/lib/x86_64-linux-gnu') + (
            ':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else ''
        )
        tessdata = LOCAL / 'usr/share/tesseract-ocr/4.00/tessdata'
        if (tessdata / 'eng.traineddata').is_file():
            env['TESSDATA_PREFIX'] = str(tessdata)
    return binary, env


def _ordered_quad(points):
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).ravel()
    return np.float32([points[sums.argmin()], points[differences.argmin()],
                       points[sums.argmax()], points[differences.argmax()]])


def _regions(rgb):
    """Prioritize small text strokes, then rectify panels; no scene/name prior."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    height, width = gray.shape
    proposals = []
    scale = min(1., 1280 / width)
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    # Light and dark lettering are both supported. A top/black-hat isolates
    # strokes rather than choosing the six largest rectangles in a scene.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (27, 11))
    masks = [np.uint8(cv2.morphologyEx(small, kind, kernel) >= 30) * 255
             for kind in (cv2.MORPH_TOPHAT, cv2.MORPH_BLACKHAT)]
    # Never merge the light characters with the dark extruded panel boundary:
    # they can otherwise become one tall component and disappear as non-text.
    for mask, join_width in [(m, j) for m in masks for j in (7, 15)]:
        joined = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (join_width, 1)))
        count, _, stats, _ = cv2.connectedComponentsWithStats(joined)
        for x, y, w, h, area in stats[1:count]:
            if not (18 <= w <= small.shape[1] * .45 and 4 <= h <= 60 and 1.3 <= w / h <= 25):
                continue
            occupancy = np.count_nonzero(mask[y:y+h, x:x+w]) / (w*h)
            if not .05 <= occupancy <= .6 or area < 20:
                continue
            if x <= 1 or y <= 1 or x+w >= small.shape[1]-1 or y+h >= small.shape[0]-1:
                continue  # Clipped text cannot establish a complete room code.
            margin = max(3, int(h * .35))
            x0, y0 = max(0, (x-margin)/scale), max(0, (y-margin)/scale)
            x1, y1 = min(width, (x+w+margin)/scale), min(height, (y+h+margin)/scale)
            points = np.float32([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
            proposals.append((float(w/h) * min(h, 18), (x0,y0,x1-x0,y1-y0), points, 7))
    edges = cv2.Canny(gray, 50, 120)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        # Extruded plaque edges can be an open thin contour. Its swept-area
        # approaches zero; the actual enclosed planar ROI is its convex hull.
        area = cv2.contourArea(cv2.convexHull(contour))
        if not max(120, height * width * .00008) < area < height * width * .12:
            continue
        quad = cv2.approxPolyDP(contour, .025 * cv2.arcLength(contour, True), True)
        if len(quad) != 4 or not cv2.isContourConvex(quad):
            # A 3D plaque has a thin side face; its visible outline can contain
            # six vertices even though the planar front is a quadrilateral.
            hull = cv2.convexHull(contour)
            quad = cv2.approxPolyDP(hull, .04 * cv2.arcLength(hull, True), True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
        x, y, box_width, box_height = cv2.boundingRect(quad)
        if box_width < 40 or box_height < 40 or not .35 < box_width / box_height < 7:
            continue
        if cv2.contourArea(quad) < 600:
            continue
        candidates.append((area, (x, y, box_width, box_height), quad.reshape(4, 2), 6))
    selected = []
    # At most four panels supplement at most eight text regions. Smaller panels
    # come first, so a distant sign is not outranked by a door or wall.
    proposals.sort(key=lambda item: -item[0])
    def panel_rank(item):
        contained_text = sum(cv2.pointPolygonTest(
            item[2].astype(np.float32), (float(bounds[0]+bounds[2]/2),
                                         float(bounds[1]+bounds[3]/2)), False) >= 0
                             for _, bounds, _, _ in proposals if bounds[3] < 65)
        return (-contained_text, item[0])
    panels = sorted(candidates, key=panel_rank)
    for rank, bounds, points, psm in proposals + panels:
        if any(np.linalg.norm(np.array(bounds) - np.array(previous[1])) < 12
               for previous in selected):
            continue
        if psm == 7 and sum(item[3] == 7 for item in selected) >= 8:
            continue
        if psm == 6 and sum(item[3] == 6 for item in selected) >= 4:
            continue
        selected.append((rank, bounds, points, psm))
        if len(selected) == 12:
            break
    selected.sort(key=lambda item: 0 if item[3] == 6 else 1)
    for _, _, points, psm in selected:
        ordered = _ordered_quad(points)
        crop_width = int(max(np.linalg.norm(ordered[1] - ordered[0]),
                             np.linalg.norm(ordered[2] - ordered[3])))
        crop_height = int(max(np.linalg.norm(ordered[3] - ordered[0]),
                              np.linalg.norm(ordered[2] - ordered[1])))
        factor = min(8., max(2., 72 / max(crop_height, 1)),
                     1400 / max(crop_width, crop_height, 1))
        out_width, out_height = max(2, int(crop_width * factor)), max(2, int(crop_height * factor))
        destination = np.float32([[0, 0], [out_width - 1, 0],
                                  [out_width - 1, out_height - 1], [0, out_height - 1]])
        transform = cv2.getPerspectiveTransform(ordered, destination)
        if abs(np.linalg.det(transform)) < 1e-12:
            continue
        crop = cv2.warpPerspective(gray, transform, (out_width, out_height),
                                   flags=cv2.INTER_CUBIC)
        if np.percentile(crop, 98) - np.percentile(crop, 2) < 30:
            continue
        if psm == 6:
            # Rectified panels often mix a room code, a pictogram and a name.
            # Isolate horizontal lettering in the rectified coordinates so
            # PSM 7 does not interpret the extruded border/logo as characters.
            foreground = np.uint8(crop > np.median(crop)+30) * 255
            grouped = cv2.morphologyEx(
                foreground, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_RECT, (max(5, out_width//25), 2)))
            count, _, stats, _ = cv2.connectedComponentsWithStats(grouped)
            letter_rows = []
            for x, y, w, h, area in stats[1:count]:
                if w < 20 or not 5 <= h <= out_height*.2 or not 1.5 < w/h < 30:
                    continue
                density = np.count_nonzero(foreground[y:y+h,x:x+w])/(w*h)
                if not .08 < density < .7:
                    continue
                letter_rows.append((x,y,w,h))
            for x,y,w,h in sorted(letter_rows, key=lambda row: row[1])[:4]:
                margin = max(3, h//3)
                x0,y0=max(0,x-margin),max(0,y-margin)
                x1,y1=min(out_width,x+w+margin),min(out_height,y+h+margin)
                row_scale = min(5., 80/max(h,1), 1400/max(x1-x0,1))
                row = cv2.resize(crop[y0:y1,x0:x1], None, fx=row_scale, fy=row_scale,
                                 interpolation=cv2.INTER_CUBIC)
                row_inverse = np.linalg.inv(transform) @ np.float64([
                    [1/row_scale,0,x0],[0,1/row_scale,y0],[0,0,1]])
                yield row, row_inverse, 7
        yield crop, np.linalg.inv(transform), psm


def _valid_word(text):
    text = text.strip(',:;!\"\'()[]{}')
    if ROOM_CODE.fullmatch(text):
        return text
    text = text.strip('.')
    if text.lower() in GENERIC_WORDS:
        return None
    if re.fullmatch(r'[A-Za-z]{3,20}', text) and re.search(r'[aeiouyAEIOUY]', text):
        return text
    return None


def _parse_lines(tsv, inverse, image_shape):
    groups = {}
    rejected = 0
    for row in csv.DictReader(io.StringIO(tsv), delimiter='\t'):
        if row.get('level') != '5' or not row.get('text', '').strip():
            continue
        try:
            confidence = float(row['conf'])
            word = _valid_word(row['text'])
            threshold = NUMERIC_MIN_CONFIDENCE if word and ROOM_CODE.fullmatch(word) else MIN_CONFIDENCE
            if confidence < threshold or word is None:
                rejected += 1
                continue
            bounds = [int(row[k]) for k in ('left', 'top', 'width', 'height')]
        except (ValueError, KeyError):
            rejected += 1
            continue
        key = tuple(row[k] for k in ('block_num', 'par_num', 'line_num'))
        groups.setdefault(key, []).append((word, confidence, bounds))
    height, width = image_shape[:2]
    lines = []
    for words in groups.values():
        # Room codes and complete words are retained exactly as read. No B/O,
        # digit, room-name or scene-specific spelling substitutions are made.
        text = ' '.join(item[0] for item in words)
        if len(words) == 1 and not ROOM_CODE.fullmatch(text) and len(text) < 4:
            continue
        x0 = min(item[2][0] for item in words)
        y0 = min(item[2][1] for item in words)
        x1 = max(item[2][0] + item[2][2] for item in words)
        y1 = max(item[2][1] + item[2][3] for item in words)
        polygon = cv2.perspectiveTransform(
            np.float32([[[x0, y0], [x1, y0], [x1, y1], [x0, y1]]]), inverse,
        )[0]
        low = np.maximum([0, 0], np.floor(polygon.min(axis=0))).astype(int)
        high = np.minimum([width, height], np.ceil(polygon.max(axis=0))).astype(int)
        if np.any(high <= low):
            continue
        lines.append({
            'text': text, 'confidence': min(item[1] for item in words) / 100.,
            'word_confidences': [round(item[1] / 100., 3) for item in words],
            'room_codes': [item[0] for item in words if ROOM_CODE.fullmatch(item[0])],
            'box_xyxy': [int(low[0]), int(low[1]), int(high[0]), int(high[1])],
            'polygon_xy': polygon.astype(float).tolist(),
            'confirmed': False,
        })
    return lines, rejected


def _raw_code_boxes(tsv, inverse):
    """Unconfirmed code locations only, used to reject directory-to-door links."""
    boxes = []
    for row in csv.DictReader(io.StringIO(tsv), delimiter='\t'):
        token = row.get('text','').strip(',:;!\"\'()[]{}')
        try:
            if row.get('level') != '5' or not ROOM_CODE.fullmatch(token) or float(row['conf']) < 20:
                continue
            x,y,w,h = [int(row[k]) for k in ('left','top','width','height')]
        except (ValueError,KeyError):
            continue
        polygon = cv2.perspectiveTransform(np.float32([[[x,y],[x+w,y],[x+w,y+h],[x,y+h]]]),inverse)[0]
        boxes.append(np.r_[polygon.min(axis=0),polygon.max(axis=0)].tolist())
    return boxes


def _has_arrow(crop):
    """Conservative pixel geometry for a horizontal seven-corner arrow."""
    mask = np.uint8(crop > np.median(crop)+30)*255
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x,y,w,h=cv2.boundingRect(contour)
        if not 1.3 <= w/max(h,1) <= 4 or h < 6:
            continue
        polygon=cv2.approxPolyDP(contour,.04*cv2.arcLength(contour,True),True)
        if not 6 <= len(polygon) <= 9 or cv2.isContourConvex(polygon):
            continue
        points=(polygon.reshape(-1,2)-[x,y])/[max(w-1,1),max(h-1,1)]
        for direction in (1,-1):
            positions=points.copy()
            if direction == -1:positions[:,0]=1-positions[:,0]
            tip=np.any((positions[:,0]>.85)&(positions[:,1]>.25)&(positions[:,1]<.75))
            wings=np.any((positions[:,0]>.35)&(positions[:,0]<.85)&(positions[:,1]<.15)) and \
                  np.any((positions[:,0]>.35)&(positions[:,0]<.85)&(positions[:,1]>.85))
            stem=np.sum((positions[:,0]<.2)&(positions[:,1]>.15)&(positions[:,1]<.85))>=2
            if tip and wings and stem:
                # A wheelchair/person pictogram can also contain seven corners.
                # Require its actual filled silhouette to match a solid arrow,
                # rather than classifying a few corner extrema as direction.
                observed=np.zeros((64,64),np.uint8)
                cv2.fillPoly(observed,[np.rint(positions*63).astype(np.int32)],1)
                best_overlap=0.
                for head_start in (.45,.55,.65):
                    for half_stem in (.12,.18,.24):
                        reference=np.zeros_like(observed)
                        template=np.float32([[0,.5-half_stem],[head_start,.5-half_stem],
                                             [head_start,0],[1,.5],[head_start,1],
                                             [head_start,.5+half_stem],[0,.5+half_stem]])
                        cv2.fillPoly(reference,[np.rint(template*63).astype(np.int32)],1)
                        intersection=np.count_nonzero(observed & reference)
                        union=np.count_nonzero(observed | reference)
                        best_overlap=max(best_overlap,intersection/max(1,union))
                if best_overlap >= .75:
                    return True
    return False


def _attach_context(lines, contexts, raw_codes):
    for context in contexts:
        inside=[]
        for code_box in raw_codes:
            code_center=(np.array(code_box[:2])+code_box[2:])/2
            if cv2.pointPolygonTest(context['polygon'],tuple(code_center),False)<0:
                continue
            if not any(_box_matches(previous,code_box) for previous in inside):
                inside.append(code_box)
        context['code_location_count']=len(inside)
    for index,line in enumerate(lines):
        box=np.array(line['box_xyxy'],float);center=(box[:2]+box[2:])/2
        enclosing=[context for context in contexts if cv2.pointPolygonTest(
            context['polygon'],tuple(center),False)>=0]
        if enclosing:
            directory=[item for item in enclosing if item['code_location_count']>1 or
                       item['arrow_detected']]
            context=min(directory or enclosing,key=lambda item:cv2.contourArea(item['polygon']))
            line.update(region_id=context['region_id'],sign_box_xyxy=context['box'],
                        sign_polygon_xy=context['polygon'].tolist(),
                        unconfirmed_room_code_location_count=context['code_location_count'],
                        arrow_detected=context['arrow_detected'],
                        sign_type='directional' if context['code_location_count']>1 or context['arrow_detected'] else
                                  'single_sign_candidate')
        else:
            line.update(region_id=f'text_region_{index}',sign_box_xyxy=line['box_xyxy'],
                        unconfirmed_room_code_location_count=0,arrow_detected=False,
                        sign_type='unclassified_text')


def recognize_signs(rgb, time_budget=2.5):
    binary, env = tesseract_environment()
    base = dict(engine='Tesseract English LSTM', uses_scene_prior=False,
                minimum_word_confidence=MIN_CONFIDENCE / 100.,
                minimum_room_code_confidence=NUMERIC_MIN_CONFIDENCE / 100.,
                box_coordinate_space='input_image_pixels',
                input_dimensions=[rgb.shape[1], rgb.shape[0]],
                room_code_requires_preprocessing_agreement=True, lines=[])
    if not binary:
        return dict(base, state='unavailable', reason='Tesseract is not installed')
    start = time.monotonic()
    lines, rejected, tested = [], 0, 0
    contexts, raw_codes = [], []
    regions = list(_regions(rgb))
    # Rectify visible rectangular sign panels first. A sparse whole-image pass
    # covers front-facing text only when rectangle searches found no valid text.
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    full_scale = min(2., 1600 / max(gray.shape))
    full = cv2.resize(gray, None, fx=full_scale, fy=full_scale,
                      interpolation=cv2.INTER_CUBIC)
    full_inverse = np.diag([1 / full_scale, 1 / full_scale, 1.]).astype(float)
    regions.append((full, full_inverse, 11))
    for index, (crop, inverse, psm) in enumerate(regions):
        if psm == 6:
            corners=np.float32([[[0,0],[crop.shape[1]-1,0],
                                 [crop.shape[1]-1,crop.shape[0]-1],[0,crop.shape[0]-1]]])
            polygon=cv2.perspectiveTransform(corners,inverse)[0]
            contexts.append(dict(region_id=f'panel_{index}',polygon=polygon,
                                 box=np.r_[polygon.min(axis=0),polygon.max(axis=0)].astype(int).tolist(),
                                 arrow_detected=_has_arrow(crop)))
        # Tesseract 4 LSTM prefers dark text on a light background. Preserve a
        # grayscale pass and a separate Otsu pass; digits must agree in both.
        normalized = 255-crop if np.median(crop) < 128 else crop
        binary_crop = cv2.threshold(normalized, 0, 255,
                                    cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        variants = []
        for variant in (normalized, binary_crop):
            variants.append(cv2.copyMakeBorder(variant, 10, 10, 10, 10,
                                               cv2.BORDER_CONSTANT, value=255))
        border_inverse = inverse @ np.float64([[1,0,-10],[0,1,-10],[0,0,1]])
        outputs = []
        numeric_hint = False
        for variant in variants:
            remaining = time_budget - (time.monotonic() - start)
            if remaining < .1:
                break
            encoded = cv2.imencode('.png', variant)[1].tobytes()
            command = [binary, 'stdin', 'stdout', '-l', 'eng', '--oem', '1',
                       '--psm', str(psm), '-c', 'user_defined_dpi=300',
                       '-c', 'tessedit_char_whitelist='+CHARACTERS, 'tsv']
            try:
                completed = subprocess.run(command, input=encoded, stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE, env=env,
                                           timeout=min(.5, remaining), check=True)
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, OSError):
                continue
            accepted, count = _parse_lines(completed.stdout.decode('utf-8', errors='replace'),
                                           border_inverse, rgb.shape)
            raw_codes.extend(_raw_code_boxes(completed.stdout.decode('utf-8',errors='replace'),
                                             border_inverse))
            for row in csv.DictReader(io.StringIO(completed.stdout.decode('utf-8', errors='replace')),
                                      delimiter='\t'):
                token = row.get('text', '').strip(',:;!\"\'()[]{}')
                if re.fullmatch(r'[A-Za-z0-9.-]+', token) and \
                    sum(character.isdigit() for character in token) >= 2 and \
                    ('.' in token or '-' in token):
                    numeric_hint = True
            tested += 1
            rejected += count
            outputs.append(accepted)
        if psm == 7 and numeric_hint:
            # Oversized interpolated digits can be less readable to LSTM.
            # Two independent moderate scales must return the exact same code
            # with >=75% word confidence; no candidate-number dictionary is used.
            for numeric_height in (48, 64):
                remaining = time_budget-(time.monotonic()-start)
                if remaining < .1:
                    break
                factor = numeric_height / normalized.shape[0]
                numeric = cv2.resize(normalized, None, fx=factor, fy=factor,
                                     interpolation=cv2.INTER_CUBIC)
                numeric = cv2.copyMakeBorder(numeric,10,10,10,10,
                                             cv2.BORDER_CONSTANT,value=255)
                numeric_inverse = inverse @ np.float64([
                    [1/factor,0,-10/factor],[0,1/factor,-10/factor],[0,0,1]])
                command = [binary,'stdin','stdout','-l','eng','--oem','1','--psm','7',
                           '-c','tessedit_char_whitelist=0123456789.-',
                           '-c','load_system_dawg=0','-c','load_freq_dawg=0','tsv']
                try:
                    completed = subprocess.run(command,input=cv2.imencode('.png',numeric)[1].tobytes(),
                                               stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=env,
                                               timeout=min(.5,remaining),check=True)
                except (subprocess.TimeoutExpired,subprocess.CalledProcessError,OSError):
                    continue
                accepted,count=_parse_lines(completed.stdout.decode('utf-8',errors='replace'),
                                            numeric_inverse,rgb.shape)
                raw_codes.extend(_raw_code_boxes(completed.stdout.decode('utf-8',errors='replace'),
                                                 numeric_inverse))
                outputs.append(accepted);tested+=1;rejected+=count
        accepted = []
        for pass_index, output in enumerate(outputs):
            for line in output:
                others = [item for other_index, other in enumerate(outputs)
                          if other_index != pass_index for item in other
                          if item['text'] == line['text']]
                if line['room_codes'] and not others:
                    rejected += 1
                    continue
                if not line['room_codes'] and not others and line['confidence'] < .9:
                    rejected += 1
                    continue
                line['preprocessing_agreement'] = bool(others)
                if others:
                    line['confidence'] = min(line['confidence'], others[0]['confidence'])
                accepted.append(line)
        for line in accepted:
            if any(old['text'] == line['text'] and
                   np.linalg.norm(np.array(old['box_xyxy']) - line['box_xyxy']) < 30
                   for old in lines):
                continue
            lines.append(line)
        if psm == 6 and any(line['room_codes'] for line in lines):
            # Finish the complete plaque (including unconfirmed directory rows),
            # then stop searching unrelated walls/fixtures in this source frame.
            break
    _attach_context(lines,contexts,raw_codes)
    return dict(base, state='single_frame_candidates' if lines else 'no_readable_text', lines=lines[:8],
                rejected_word_count=rejected, regions_tested=tested,
                processing_ms=round((time.monotonic() - start) * 1000, 2))


def _box_matches(left, right):
    a, b = np.array(left, float), np.array(right, float)
    intersection = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2])).prod()
    areas = (a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()
    iou = intersection / max(1., areas-intersection)
    centers = np.linalg.norm((a[:2]+a[2:])/2-(b[:2]+b[2:])/2)
    return iou >= .2 or centers < max(12., .3*min(np.linalg.norm(a[2:]-a[:2]),
                                                  np.linalg.norm(b[2:]-b[:2])))


class SignConfirmation:
    """Exact OCR spelling in consecutive distinct source frames, never guesses."""
    def __init__(self, required=3, max_gap=6.5):
        self.required = max(2, int(required))
        self.max_gap = max_gap
        self.previous = []
        self.previous_stamp = None
        self.next_id = 1

    def update(self, result, stamp):
        stamp = float(stamp)
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            return dict(result, lines=[], state='duplicate_or_out_of_order_frame',
                        temporal_confirmation=True, required_distinct_frames=self.required)
        valid_previous = self.previous if self.previous_stamp is not None and \
            0 < stamp-self.previous_stamp <= self.max_gap else []
        tracks, confirmed = [], []
        for line in result.get('lines', []):
            match = next((old for old in valid_previous if old['text'] == line['text'] and
                          _box_matches(old['box_xyxy'], line['box_xyxy'])), None)
            if match:
                record = dict(line, confirmation_frames=match['confirmation_frames']+1,
                              track_id=match['track_id'], first_seen_stamp=match['first_seen_stamp'],
                              last_seen_stamp=stamp,
                              confidence=min(line['confidence'], match['confidence']))
                # Directory evidence is classification context only, never a
                # room identity. Keep it when later OCR temporarily loses a row.
                if match.get('sign_type') == 'directional':
                    record['sign_type'] = 'directional'
            else:
                record = dict(line, confirmation_frames=1, track_id=self.next_id,
                              first_seen_stamp=stamp, last_seen_stamp=stamp)
                self.next_id += 1
            record['confirmed'] = record['confirmation_frames'] >= self.required
            tracks.append(record)
            if record['confirmed']:
                confirmed.append(record)
        self.previous, self.previous_stamp = tracks, stamp
        return dict(result, lines=confirmed, candidate_count=len(tracks),
                    state='readable_text' if confirmed else
                          ('confirming_text' if tracks else 'no_readable_text'),
                    temporal_confirmation=True, required_distinct_frames=self.required)


class AsyncSignOCR:
    """One low-frequency worker, one replaceable frame; submit never waits for OCR."""
    def __init__(self, interval=2., max_frame_age=2.5, confirmation_frames=3, time_budget=2.5):
        self.interval = interval
        self.max_frame_age = max_frame_age
        self.time_budget = time_budget
        self.confirmation = SignConfirmation(confirmation_frames, max_gap=max(6.5, interval*2.5))
        self.condition = threading.Condition()
        self.pending = None
        self.latest = None
        self.submitted_at = -float('inf')
        self.closed = False
        self.enabled = bool(tesseract_environment()[0])
        self.thread = threading.Thread(target=self._work, name='office-sign-ocr', daemon=True)
        if self.enabled:
            self.thread.start()

    def submit(self, rgb, frame_stamp):
        now = time.monotonic()
        with self.condition:
            if not self.enabled or self.closed or now - self.submitted_at < self.interval:
                return False
            self.submitted_at = now
            self.pending = (rgb.copy(), float(frame_stamp))
            self.condition.notify()
        return True

    def _work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                rgb, frame_stamp = self.pending
                self.pending = None
            try:
                result = self.confirmation.update(recognize_signs(rgb, self.time_budget), frame_stamp)
            except Exception as exc:
                result = dict(state='unavailable', lines=[], reason=type(exc).__name__,
                              engine='Tesseract English LSTM', uses_scene_prior=False)
            result['input_stamp'] = frame_stamp
            signatures = []
            for line in result['lines']:
                x0, y0, x1, y1 = line['box_xyxy']
                patch = cv2.cvtColor(rgb[y0:y1, x0:x1], cv2.COLOR_RGB2GRAY)
                signatures.append(cv2.resize(patch, (48, 16)))
            with self.condition:
                # Source pixels and coordinates belong to the same immutable
                # completed inference. Projection must not use a newer frame.
                rgb.flags.writeable = False
                self.latest = (result, frame_stamp, signatures, rgb.shape, rgb)

    def source_result(self):
        """Return {'rgb', 'stamp', 'result'} for the exact completed source frame.

        Returned arrays/dicts are read-only by convention; consumers must copy
        before drawing. This historic result has no current-frame TTL and must
        be paired with depth/pose at ``stamp``, never the latest sensor message.
        Only result['lines'] with confirmed=True are fit for semantic anchors.
        """
        with self.condition:
            latest = self.latest
        if latest is None:
            return None
        result, source_stamp, _, _, source_rgb = latest
        return {'rgb': source_rgb, 'stamp': source_stamp, 'result': result}

    def snapshot(self, rgb, frame_stamp):
        base = dict(state='pending' if self.enabled else 'unavailable', lines=[],
                    engine='Tesseract English LSTM', uses_scene_prior=False,
                    update_interval_seconds=self.interval, async_worker=True)
        with self.condition:
            latest = self.latest
        if latest is None:
            return base
        result, source_stamp, signatures, shape, _ = latest
        age = float(frame_stamp) - source_stamp
        base.update(result, input_stamp=source_stamp, result_age_seconds=round(age, 3))
        if not 0 <= age <= self.max_frame_age or shape != rgb.shape:
            base.update(state='stale', lines=[])
            return base
        current = []
        for line, signature in zip(result['lines'], signatures):
            x0, y0, x1, y1 = line['box_xyxy']
            patch = cv2.cvtColor(rgb[y0:y1, x0:x1], cv2.COLOR_RGB2GRAY)
            sampled = cv2.resize(patch, (48, 16))
            if np.abs(sampled.astype(float) - signature).mean() < 18:
                current.append(line)
        base['lines'] = current
        if result['lines'] and not current:
            base['state'] = 'image_changed'
        return base

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify()
        if self.thread.is_alive():
            self.thread.join(timeout=.1)


def draw_text_evidence(rgb, lines):
    image = rgb.copy()
    height, width = image.shape[:2]
    for line in lines:
        x0, y0, x1, y1 = line['box_xyxy']
        cv2.polylines(image, [np.asarray(line['polygon_xy'], np.int32)], True,
                      (255, 225, 65), 2, cv2.LINE_AA)
        text = f"OCR {line['text']} ({line['confidence']:.0%})"
        font_scale = min(1., max(.55, width / 1200))
        cv2.putText(image, text[:70], (max(0, x0), max(20, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text[:70], (max(0, x0), max(20, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 225, 65), 2, cv2.LINE_AA)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bgr = cv2.imread(str(args.image))
    if bgr is None:
        parser.error('Cannot read input image')
    rgb = bgr[:, :, ::-1]
    result = recognize_signs(rgb)
    result['source_image'] = str(args.image.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'ocr.json').write_text(json.dumps(result, indent=2) + '\n')
    cv2.imwrite(str(args.output / 'ocr.png'), draw_text_evidence(rgb, result['lines'])[:, :, ::-1])
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
