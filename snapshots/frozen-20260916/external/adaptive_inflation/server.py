from http.server import ThreadingHTTPServer,SimpleHTTPRequestHandler
from urllib.parse import urlparse,parse_qs
from pathlib import Path
import gzip,json,time,threading
import numpy as np
from adaptive_core import build_grid,analyze,Settings
R=Path(__file__).resolve().parent
META=json.loads((R/'datasets.json').read_text());DATA={};CLOUD={};LOCK=threading.Lock()
COLORS=['#8b9db1','#32b9ff','#b37cff','#ffb44c','#ff568b','#bbc7d5','#5bbda7','#e4d16d']
NAMES=['未分类','楼梯候选','斜坡候选','柱状物候选','行人候选','墙面候选','地面候选','地形待确认']
for key,meta in META.items():
    a=np.load(R/f'{key}_prepared.npz');p=a['points'];labs=a['labels'];g=build_grid(a['analysis_points'],a['analysis_labels'],meta['floor']);DATA[key]=g
    traces=[]
    for k in np.unique(labs):
        q=p[labs==k];traces.append(dict(type='scatter3d',mode='markers',name=NAMES[k] if key=='go2w' else 'PCD 观测（按高度着色）',x=q[:,0].round(4).tolist(),y=q[:,1].round(4).tolist(),z=q[:,2].round(4).tolist(),marker=dict(size=1.25,opacity=.8,color=COLORS[k] if key=='go2w' else q[:,2].round(2).tolist(),colorscale='Cividis'),hoverinfo='skip'))
    CLOUD[key]=json.dumps(dict(traces=traces,meta=meta),ensure_ascii=False,separators=(',',':')).encode()


def compute(key,params):
    meta=META[key];g=DATA[key];ex=meta['examples'][params.get('example',['narrow'])[0]]
    pos=[float(params.get('x',[ex['position'][0]])[0]),float(params.get('y',[ex['position'][1]])[0])]
    yaw=float(params.get('yaw',[ex['yaw']])[0]);speed=float(params.get('speed',[.2])[0])
    if not all(np.isfinite([*pos,yaw,speed])) or not 0<=speed<=1.2:raise ValueError('Invalid probe')
    prev=float(params['previous'][0]) if 'previous' in params else None
    if prev is not None and (not np.isfinite(prev) or prev<0):prev=None
    result,grids=analyze(g,pos,yaw,speed,previous=prev)
    state=params.get('layer',['adaptive'])[0]
    chosen=grids['inflated'] if state=='adaptive' else grids['fixed'] if state=='fixed' else g['occupancy']
    occ=g['occupancy'];rows,cols=np.nonzero(chosen&~occ)
    xs=g['origin'][0]+(np.arange(occ.shape[1])+.5)*g['res'];ys=g['origin'][1]+(np.arange(occ.shape[0])+.5)*g['res']
    # Each colored square is a derived grid cell, never presented as a sensor return.
    overlay=dict(type='scatter3d',mode='markers',name='膨胀栅格 · 水平投影',x=xs[cols].round(3).tolist(),y=ys[rows].round(3).tolist(),z=np.full(len(rows),meta['floor']+.035).tolist(),marker=dict(symbol='square',size=2.,color='#fcaa42',opacity=.30),hoverinfo='skip')
    # Four explicit classes: no return, observed XY, added inflation, measured obstacle.
    raster=np.where(g['observed'],1,0).astype(np.uint8);raster[chosen]=2;raster[occ]=3
    result.update(dataset=key,layer=state,height_band=meta['height_band'],floor=meta['floor'],resolution=g['res'])
    output=dict(metrics=result,x=xs.round(3).tolist(),y=ys.round(3).tolist(),raster=raster.tolist(),overlay=overlay)
    with LOCK:
        (R/'current_state.tmp').write_text(json.dumps(result,ensure_ascii=False,indent=2));(R/'current_state.tmp').replace(R/'current_state.json')
        np.savez_compressed(R/'current_costmap.tmp.npz',metadata=json.dumps(result),occupancy=occ,observed=g['observed'],inflated=grids['inflated'],fixed=grids['fixed'],distance=g['distance'],cost=grids['cost'],origin=g['origin'],resolution=g['res'],floor=g['floor'])
        (R/'current_costmap.tmp.npz').replace(R/'current_costmap.npz')
    return output


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(R),**kwargs)
    def log_message(self,fmt,*args):pass
    def send_data(self,data):
        if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False,separators=(',',':')).encode()
        data=gzip.compress(data,compresslevel=2)
        self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Encoding','gzip');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def do_GET(self):
        url=urlparse(self.path);q=parse_qs(url.query);key=q.get('dataset',['flycar'])[0]
        try:
            if url.path=='/api/meta':return self.send_data(META)
            if url.path=='/api/cloud':return self.send_data(CLOUD[key])
            if url.path=='/api/analyze':return self.send_data(compute(key,q))
            if url.path=='/plotly.min.js':
                data=(R.parent/'plotly.min.js').read_bytes();self.send_response(200);self.send_header('Content-Type','application/javascript');self.end_headers();self.wfile.write(data);return
            return super().do_GET()
        except (ValueError,KeyError) as e:self.send_error(400,str(e))
        except BrokenPipeError:pass
if __name__=='__main__':
    print('Adaptive inflation preview http://127.0.0.1:8877/ — no robot control',flush=True)
    ThreadingHTTPServer(('127.0.0.1',8877),Handler).serve_forever()
