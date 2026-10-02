"""Scene-prior labels for observed points; not a learned semantic segmenter."""
import math
import xml.etree.ElementTree as ET
import numpy as np

CLASSES = {0:'unassigned',1:'stairs',2:'ramp',3:'static_cylinder',4:'dynamic_proxy',5:'ground'}
COLORS = np.array([[142,157,177],[45,178,241],[55,208,145],[255,161,52],[236,81,177],[114,133,156]],dtype=np.uint8)

def rotation(euler):
    x,y,z=euler;cx,sx=math.cos(x),math.sin(x);cy,sy=math.cos(y),math.sin(y);cz,sz=math.cos(z),math.sin(z)
    return np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]]) @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]) @ np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])

class SceneSemantics:
    def __init__(self,path):
        self.objects=[]
        for g in ET.parse(path).getroot().findall('./worldbody/geom'):
            name=g.get('name','');typ=g.get('type','sphere')
            if g.get('contype')=='0' and g.get('conaffinity')=='0':continue
            label=1 if name.startswith(('box_OBS','box2_OBS')) else 2 if name.startswith('extended_ramp_') else 3 if typ=='cylinder' else 0
            if not label:continue
            self.objects.append({'name':name,'class_id':label,'type':typ,'position':list(map(float,g.get('pos','0 0 0').split())),'size':list(map(float,g.get('size','').split())),'euler':list(map(float,g.get('euler','0 0 0').split()))})
    def label(self,points,actors=(),tolerance=.10):
        labels=np.zeros(len(points),dtype=np.uint8)
        labels[np.abs(points[:,2])<.06]=5
        for o in self.objects:
            p=points-np.array(o['position']);s=o['size']
            if o['type']=='box':
                local=p@rotation(o['euler']);q=np.abs(local)-np.array(s)
                hit=(np.max(q,axis=1)<=tolerance)&(np.min(np.abs(q),axis=1)<=tolerance)
            else:
                hit=(np.abs(np.linalg.norm(p[:,:2],axis=1)-s[0])<=tolerance)&(np.abs(p[:,2])<=s[1]+tolerance)
            labels[hit]=o['class_id']
        for a in actors:
            hit=(np.abs(np.linalg.norm(points[:,:2]-[a['x'],a['y']],axis=1)-.22)<=tolerance)&(points[:,2]>=0)&(points[:,2]<=1.8)
            labels[hit]=4
        return labels
