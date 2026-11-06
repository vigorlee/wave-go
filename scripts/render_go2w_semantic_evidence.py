#!/usr/bin/python3
"""Render captured observations without synthesizing missing sensor returns."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image
from go2w_semantic_geometry import CLASSES,COLORS
NAMES={'stairs_up':'上楼梯避障','stairs_down':'下楼梯避障','flat_slalom':'平地交错圆柱','ramp_up':'上坡避障','ramp_down':'下坡避障','dynamic_crossing':'动态障碍通行'}
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10,'figure.facecolor':'#f4f7fb','axes.facecolor':'#ffffff','axes.edgecolor':'#cad4df','text.color':'#172c43','axes.labelcolor':'#334b64','xtick.color':'#526981','ytick.color':'#526981'})

def roi(p,center,dy=3.):return (np.abs(p[:,0]-center[0])<2.6)&(np.abs(p[:,1]-center[1])<dy)&(p[:,2]>-.15)&(p[:,2]<3.2)
def bounded(p,maxn=32000):return p[::max(1,int(np.ceil(len(p)/maxn)))]
def ply(path,points,labels):
    c=COLORS[labels];data=np.column_stack([points,c,labels]);header='ply\nformat ascii 1.0\nelement vertex '+str(len(points))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar label\nend_header\n'
    with path.open('w') as f:f.write(header);np.savetxt(f,data,fmt=['%.5f']*3+['%d']*4)

def render(snapshot,dest):
    meta=json.loads((snapshot/'metadata.json').read_text());a=np.load(snapshot/'clouds.npz');p=a['robot'];region=meta['region'];center=p.copy()
    if region=='stairs_up':center[1]=2.5
    elif region=='stairs_down':center[1]=7.5
    mask=roi(a['raw_world'],center);raw=a['raw_world'][mask];labels=a['raw_labels'][mask];stride=max(1,int(np.ceil(len(raw)/32000)));raws=raw[::stride];labs=labels[::stride]
    fused=a['fused_world'];fm=roi(fused,center);world=meta['world'].get('payload',{});static=int(world.get('static_modeled_points',0));dynamic=int(world.get('dynamic_modeled_points',0));split=max(0,len(fused)-static-dynamic)
    dense=np.load(snapshot/'dense_depth.npz')['points'] if (snapshot/'dense_depth.npz').exists() else None
    if dense is not None:dense=bounded(dense[roi(dense,center)],60000)
    algorithm=np.load(snapshot/'algorithm.npz') if (snapshot/'algorithm.npz').exists() else None
    fig=plt.figure(figsize=(16,11));gs=fig.add_gridspec(2,2,left=.045,right=.975,bottom=.15,top=.86,wspace=.14,hspace=.25)
    ax=fig.add_subplot(gs[0,0]);ax.set_title('A  同期相机观测',loc='left',weight='bold',pad=10)
    if (snapshot/'camera.jpg').exists():ax.imshow(Image.open(snapshot/'camera.jpg'))
    ax.axis('off')
    for slot,title,semantic in [(gs[0,1],'B  密集深度点云 · 按高度着色' if dense is not None else 'B  原始雷达点云 · 按高度着色',False),(gs[1,0],'C  实时算法 · 地形候选与障碍跟踪' if algorithm is not None else 'C  原始观测点 · 场景先验语义关联',True)]:
        ax=fig.add_subplot(slot,projection='3d');ax.set_title(title,loc='left',weight='bold',pad=12)
        if semantic:
            if algorithm is not None:
                from go2w_live_perception import LABELS,PALETTE
                valid=roi(algorithm['points'],center);plotp=algorithm['points'][valid];plotl=algorithm['labels'][valid];plotclasses=LABELS;plotcolors=PALETTE
            else:plotp=raws;plotl=labs;plotclasses=CLASSES;plotcolors=COLORS
            for k in plotclasses:
                use=plotl==k
                if use.any():ax.scatter(plotp[use,0],plotp[use,1],plotp[use,2],c=[plotcolors[k]/255],s=2 if k in (0,5) else 5,alpha=.48 if k in (0,5) else .95,depthshade=False)
        else:
            bp=dense if dense is not None else raws
            pts=ax.scatter(bp[:,0],bp[:,1],bp[:,2],c=bp[:,2],cmap='viridis',s=2,vmin=0,vmax=2.5,depthshade=False);fig.colorbar(pts,ax=ax,shrink=.55,pad=.06,label='高度 / m')
        if semantic and algorithm is not None:
            for obj in meta.get('algorithm',{}).get('status',{}).get('payload',{}).get('objects',[]):
                c=np.array(obj['center'])
                if not roi(c[None,:],center)[0]:continue
                lo=np.array(obj['low']);hi=np.array(obj['high']);corners=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])]);color='#ee3b82' if obj['label']=='moving cluster' else '#de871b'
                for i in range(8):
                    for bit in (1,2,4):
                        j=i^bit
                        if j>i:ax.plot(*corners[[i,j]].T,color=color,lw=.9)
                ax.text(c[0],c[1],hi[2]+.13,f"#{obj['id']} {obj['speed_mps']:.2f} m/s",fontsize=7,color=color)
        ax.scatter(*p,c='#e23e57',s=75,marker='*',depthshade=False);ax.set(xlim=(center[0]-2.6,center[0]+2.6),ylim=(center[1]-3,center[1]+3),zlim=(0,2.7),xlabel='x / m',ylabel='y / m',zlabel='z / m' if semantic else '');ax.set_box_aspect((5.2,6,2.7));ax.view_init(elev=28,azim=-52)
    ax=fig.add_subplot(gs[1,1]);ax.set_title('D  导航融合点云 · 来源与实际轨迹',loc='left',weight='bold',pad=12)
    sensors=bounded(fused[:split][fm[:split]],18000);ax.scatter(sensors[:,0],sensors[:,1],s=1,c='#aab7c4',alpha=.4,label='传感器点')
    model=fused[split:split+static];model=model[roi(model,center)];ax.scatter(model[:,0],model[:,1],s=5,c='#efa237',label='静态模型补点')
    moving=fused[split+static:];moving=moving[roi(moving,center)];ax.scatter(moving[:,0],moving[:,1],s=5,c='#db4fa0',label='动态状态补点')
    tr=a['trajectory'];tr=tr[(np.abs(tr[:,2]-center[1])<3.5)&(np.abs(tr[:,1]-center[0])<3)]
    ax.plot(tr[:,1],tr[:,2],c='#6554c0',lw=2,label='已行走轨迹');ax.scatter(p[0],p[1],s=110,c='#e23e57',marker='*',zorder=5,label='机器狗中心')
    ax.set(xlim=(center[0]-2.6,center[0]+2.6),ylim=(center[1]-3,center[1]+3),xlabel='x / m',ylabel='y / m');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8,framealpha=.9)
    if algorithm is not None:
        from go2w_live_perception import PALETTE
        legend_colors=PALETTE;legend_names={0:'未分类',1:'地面/平台',2:'楼梯候选',3:'坡面候选',4:'墙/结构',5:'障碍簇',6:'移动簇'}
    else:legend_colors=COLORS;legend_names={0:'未关联',1:'楼梯',2:'坡道',3:'静态圆柱',4:'动态代理',5:'地面'}
    handles=[Line2D([0],[0],marker='o',color='none',markerfacecolor=legend_colors[k]/255,label=legend_names[k],markersize=7) for k in legend_names]
    fig.legend(handles=handles,loc='lower left',bbox_to_anchor=(.045,.092),ncol=7,frameon=False)
    fig.suptitle('Go2-W  |  '+NAMES[region]+'：点云与语义信息',x=.045,y=.965,ha='left',fontsize=21,weight='bold')
    fig.text(.045,.902,f'采集 {snapshot.name}   ·   原始 ROI {len(raw):,} 点 / {len(meta["raw_frames"])} 帧   ·   坐标系 odom（米）   ·   相机接收年龄 {meta.get("camera",{}).get("age_seconds",0):.2f} s',fontsize=10,color='#526981')
    stage=meta.get('mission',{}).get('payload',{}).get('stage_id','unknown');active=world.get('crossing_yield',{}).get('active',False)
    source='实时点云几何＋时序跟踪，不使用场景先验' if algorithm is not None else '场景先验几何关联，非学习式分割'
    fig.text(.045,.063,f'任务阶段：{stage}   |   动态等待：{active}   |   C 图来源：{source}',fontsize=10)
    fig.text(.045,.032,'B / C 只显示实际接收的原始点；D 单独区分传感器与模型补点。密集深度 B 与算法 C 为近时独立帧；无深度数据时 B 显示累积雷达点。原始数据均保留。',fontsize=10,color='#526981')
    dest.mkdir(exist_ok=True,parents=True);out=dest/(region+'.png');fig.savefig(out,dpi=150);plt.close(fig);ply(dest/(region+'_raw_semantic.ply'),raw,labels)
    return {'region':region,'snapshot':snapshot.name,'image':out.name,'raw_roi_points':len(raw),'roi_class_counts':{CLASSES[k]:int((labels==k).sum()) for k in CLASSES},'camera_age_seconds':meta.get('camera',{}).get('age_seconds'),'capture_time':meta['captured_at']}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('pointcloud',type=Path);args=ap.parse_args();out=args.pointcloud.parent/'pointcloud_figures';selected=[]
    for region in NAMES:
        choices=[]
        for d in args.pointcloud.glob(region+'_*'):
            if not (d/'metadata.json').exists():continue
            m=json.loads((d/'metadata.json').read_text());key='stairs' if 'stairs' in region else 'ramp' if 'ramp' in region else 'static_cylinder' if region=='flat_slalom' else 'dynamic_proxy';score=m['class_counts'].get(key,0)
            if m.get('camera',{}).get('age_seconds',999)>.8:continue
            if region=='dynamic_crossing' and m['world'].get('payload',{}).get('crossing_yield',{}).get('active'):score+=100000
            choices.append((score,d))
        if choices:selected.append(render(max(choices,key=lambda t:t[0])[1],out))
    (out/'selection.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2));print(json.dumps(selected,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
