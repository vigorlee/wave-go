"""Conservative display filtering; no simulator labels or coordinates are used."""
import re
import time
import numpy as np

CLASSES = {'stairs':1, 'ramp':2, 'cylinder':3, 'person':4, 'wall':5, 'floor':6}
THRESHOLDS = {1:.32, 2:.36, 3:.48, 4:.48, 5:.40, 6:.38, 7:.40}

def classify(text):
    found = {value for key,value in CLASSES.items() if re.search(r'\b'+key+r'\b', str(text).lower())}
    if found == {1,2}: return 7
    return next(iter(found)) if len(found)==1 else 0

def iou(a,b):
    a,b=np.asarray(a),np.asarray(b)
    intersection=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])).prod()
    return float(intersection/max(1e-9,np.maximum(0,a[2:]-a[:2]).prod()+np.maximum(0,b[2:]-b[:2]).prod()-intersection))

class DetectionFilter:
    def __init__(self, temporal=False):
        self.temporal=temporal
        self.previous=[]
        self.last_time=None

    def select(self, boxes, scores, names, shape, now=None):
        now=time.monotonic() if now is None else now
        if self.last_time is None or now-self.last_time>1.: self.previous=[]
        h,w=shape[:2];selected=[];audit=[];current=[]
        for index in np.argsort(-np.asarray(scores)):
            box=np.asarray(boxes[index],float);score=float(scores[index]);label=classify(names[index]);reason=None
            bw,bh=box[2]-box[0],box[3]-box[1]
            if not np.isfinite(box).all() or not np.isfinite(score) or bw<=0 or bh<=0: reason='invalid_box'
            elif not label: reason='ambiguous_text'
            elif score<THRESHOLDS[label]: reason='class_confidence'
            elif label==4 and (bh/max(bw,1)<1.15 or bw*bh>w*h*.4): reason='person_box_shape'
            elif any(iou(box,boxes[j])>.55 for j in selected): reason='duplicate_box'
            if reason is None:
                hits=1+max((n for old_label,old_box,n in self.previous if old_label==label and iou(box,old_box)>.3),default=0)
                current.append((label,box,min(hits,3)))
                if self.temporal and hits<2: reason='awaiting_second_frame'
                else: selected.append(int(index))
            audit.append({'raw_label':str(names[index]),'score':score,'box_xyxy':box.tolist(),'class_id':label,'accepted':reason is None,'reason':reason})
        self.previous=current;self.last_time=now
        return selected[:8],audit
