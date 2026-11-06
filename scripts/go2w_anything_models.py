#!/usr/bin/env python3
"""Actual pretrained Depth Anything V2 + Grounding DINO + SAM 2 inference."""
import time,json
from pathlib import Path
import numpy as np
import torch
import cv2
from go2w_detection_filter import DetectionFilter,classify
from PIL import Image
from transformers import AutoImageProcessor,AutoModelForDepthEstimation,AutoProcessor,AutoModelForZeroShotObjectDetection,Sam2Processor,Sam2Model

ROOT=Path(__file__).resolve().parents[1]
PROMPT='stairs. ramp. cylinder. person. wall. floor. door.'
COLORS=np.array([[85,100,115],[45,170,255],[180,105,250],[255,165,35],[250,65,135],[155,175,195],[65,195,135],[235,205,65],[55,225,225]],np.uint8)
LABELS=['unassigned','stairs','ramp','cylinder','person','wall','floor','stairs/ramp ambiguous','door']
LABELS_ZH=['未分类','楼梯','坡道','圆柱障碍物','行人','墙面','地面','楼梯/坡道待定','门']

def chinese_label(class_id, raw_label=''):
 """Translate the accepted model class for display, keeping raw_label intact."""
 try:
  class_id=int(class_id)
  if 0 <= class_id < len(LABELS_ZH): return LABELS_ZH[class_id]
 except (TypeError, ValueError): pass
 return str(raw_label)

class Anything:
 def __init__(self,root=ROOT/'models/anything',temporal=False):
  self.filter=DetectionFilter(temporal)
  torch.set_num_threads(4);self.device='cuda' if torch.cuda.is_available() else 'cpu'
  self.dp=AutoImageProcessor.from_pretrained(root/'depth-anything-v2-small',local_files_only=True)
  self.dm=AutoModelForDepthEstimation.from_pretrained(root/'depth-anything-v2-small',local_files_only=True).to(self.device).eval()
  self.gp=AutoProcessor.from_pretrained(root/'grounding-dino-tiny',local_files_only=True)
  self.gm=AutoModelForZeroShotObjectDetection.from_pretrained(root/'grounding-dino-tiny',local_files_only=True,disable_custom_kernels=True).to(self.device).eval()
  self.sp=Sam2Processor.from_pretrained(root/'sam2.1-hiera-tiny',local_files_only=True)
  self.sm=Sam2Model.from_pretrained(root/'sam2.1-hiera-tiny',local_files_only=True).to(self.device).eval()
 @torch.inference_mode()
 def infer(self,rgb,prompt=PROMPT,detection_filter=None):
  started=time.perf_counter();im=Image.fromarray(rgb);h,w=rgb.shape[:2]
  di=self.dp(images=im,return_tensors='pt').to(self.device);d=self.dm(**di).predicted_depth
  depth=torch.nn.functional.interpolate(d[:,None],size=(h,w),mode='bicubic',align_corners=False)[0,0].cpu().numpy()
  depth_ms=(time.perf_counter()-started)*1000;t=time.perf_counter()
  gi=self.gp(images=im,text=prompt,return_tensors='pt').to(self.device);go=self.gm(**gi)
  det=self.gp.post_process_grounded_object_detection(go,gi.input_ids,threshold=.25,text_threshold=.2,target_sizes=[(h,w)])[0]
  boxes=det['boxes'].cpu().numpy();scores=det['scores'].cpu().numpy();names=det.get('text_labels',det.get('labels',[]));active_filter=detection_filter if detection_filter is not None else self.filter;order,audit=active_filter.select(boxes,scores,names,rgb.shape)
  boxes=boxes[order];scores=scores[order];names=[names[i] for i in order];dino_ms=(time.perf_counter()-t)*1000;t=time.perf_counter()
  sem=np.zeros((h,w),np.uint8);objects=[]
  if len(boxes):
   si=self.sp(images=im,input_boxes=[boxes.tolist()],return_tensors='pt').to(self.device)
   so=self.sm(**si,multimask_output=False)
   masks=self.sp.post_process_masks(so.pred_masks.cpu(),si['original_sizes'])[0][:,0].numpy().astype(bool)
   quality=so.iou_scores[0,:,0].cpu().numpy()
   # Resolve overlapping masks by model confidence, not scene knowledge.
   paint_order=np.argsort(scores)
   for i in paint_order:
    if masks[i].sum()<max(128,h*w*.0005):continue
    label=classify(names[i])
    if quality[i]<.65:continue
    sem[masks[i]]=label
    objects.append({'label':str(names[i]),'label_zh':chinese_label(label,names[i]),'class_id':label,'detection_score':float(scores[i]),'sam_iou_prediction':float(quality[i]),'box_xyxy':boxes[i].tolist(),'mask_pixels':int(masks[i].sum())})
  sam_ms=(time.perf_counter()-t)*1000
  return depth,sem,objects,dict(depth_ms=depth_ms,dino_ms=dino_ms,sam_ms=sam_ms,total_ms=(time.perf_counter()-started)*1000,device=self.device,prompt=prompt,detection_filter_audit=audit,temporal_confirmation=active_filter.temporal,semantic_source='GroundingDINO_text_detection_plus_SAM2_masks',depth_source='DepthAnythingV2_relative_inverse_depth',uses_scene_prior=False)

def panel(rgb,depth,sem,objects,stats,door_view=None):
 from go2w_sign_ocr import draw_text_evidence
 ocr=stats.get('ocr');rgb_evidence=draw_text_evidence(rgb,ocr.get('lines',[])) if ocr else rgb
 overlay=(rgb*.6+COLORS[sem]*.4).astype(np.uint8);overlay[sem==0]=rgb[sem==0]
 for o in objects:
  x1,y1,x2,y2=map(int,o['box_xyxy']);c=tuple(map(int,COLORS[o['class_id']]))
  # Hershey fonts support Latin labels; preserve Chinese only in metadata.
  display_label=LABELS[int(o['class_id'])]
  cv2.rectangle(overlay,(x1,y1),(x2,y2),c,2);cv2.putText(overlay,f"{display_label} {o['detection_score']:.2f}",(x1,max(18,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,c,1,cv2.LINE_AA)
 lo,hi=np.percentile(depth,[2,98]);vis=cv2.applyColorMap((np.clip((depth-lo)/max(1e-6,hi-lo),0,1)*255).astype(np.uint8),cv2.COLORMAP_INFERNO)[:,:,::-1]
 tiles=[]
 for title,img in [('Room plaque - live sign camera' if door_view is not None else 'RGB - live camera + OCR',door_view if door_view is not None else rgb_evidence),('Depth Anything V2 - primary camera',vis),('Grounding DINO + SAM 2 - primary',overlay),('Learned semantic masks - primary',COLORS[sem])]:
  tile=np.zeros((310,480,3),np.uint8);tile[:40]=[12,20,32]
  fit=min(480/img.shape[1],270/img.shape[0]);scaled=cv2.resize(img,(max(1,round(img.shape[1]*fit)),max(1,round(img.shape[0]*fit))))
  x=(480-scaled.shape[1])//2;y=40+(270-scaled.shape[0])//2;tile[y:y+scaled.shape[0],x:x+scaled.shape[1]]=scaled
  cv2.putText(tile,title,(12,26),cv2.FONT_HERSHEY_SIMPLEX,.55,(230,240,250),1,cv2.LINE_AA);tiles.append(tile)
 target=stats.get('door_target');canvas=np.vstack([np.hstack(tiles[:2]),np.hstack(tiles[2:])]);footer=np.full((140 if target else (76 if ocr else 50),960,3),[12,20,32],np.uint8)
 cv2.putText(footer,f"PRETRAINED MODELS | {stats['total_ms']:.0f} ms | {len(objects)} detections | depth is NOT metric",(12,29),cv2.FONT_HERSHEY_SIMPLEX,.55,(100,235,190),1,cv2.LINE_AA)
 if ocr:
  text=' | '.join(f"{item['text']} ({item['confidence']:.0%})" for item in ocr.get('lines',[])[:3])
  cv2.putText(footer,('OCR: '+(text or ocr.get('state','pending')))[:100],(12,58),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,225,65),1,cv2.LINE_AA)
 if target:
  room_ocr=target.get('ocr',{});goal=target.get('goal_status',{});room_text=' | '.join(line['text'] for line in room_ocr.get('lines',[]) if line.get('room_codes'))
  cv2.putText(footer,('ROOM OCR: '+(room_text or 'room number pending'))[:100],(12,82),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,225,65),1,cv2.LINE_AA)
  room_codes=', '.join(goal.get('observed_room_codes',[]));state=goal.get('state','pending');color=(100,245,150) if goal.get('confirmed') else (255,190,90)
  cv2.putText(footer,('TARGET '+room_codes+' | '+state)[:100],(12,108),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1,cv2.LINE_AA)
  if 'sign_position' in goal:
   p=goal['sign_position'];cv2.putText(footer,f"MEASURED PLATE {goal.get('frame','')} ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m",(12,132),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1,cv2.LINE_AA)
 return np.vstack([canvas,footer])

if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();a.add_argument('image',type=Path);a.add_argument('--output',type=Path,required=True);args=a.parse_args();args.output.mkdir(parents=True,exist_ok=True)
 engine=Anything();rgb=np.array(Image.open(args.image).convert('RGB'));d,s,o,t=engine.infer(rgb)
 Image.fromarray(panel(rgb,d,s,o,t)).save(args.output/'models.png');np.savez_compressed(args.output/'predictions.npz',relative_depth=d,semantic_labels=s)
 (args.output/'result.json').write_text(json.dumps({'objects':o,**t},indent=2));print(json.dumps({'objects':o,**t}))
