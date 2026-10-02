"""One X11 screen frame, four crops: a shared recording clock for all panels."""
from pathlib import Path
import subprocess,re,json,time,shlex,sys
R=Path(sys.argv[1]).resolve();R.mkdir(parents=True,exist_ok=True)
patterns=['MuJoCo Physics |','office_demo.rviz','Depth Anything V2 + Grounded SAM 2','zsibot_mujoco_ue (64-bit']
titles=['01  MuJoCo | Robot motion','02  RViz | Dense 3D map + Dijkstra / A-star','03  Depth Anything + Grounded SAM2','04  UE | Native walking pedestrians']
if len(sys.argv)>2:
 titles[1]='02  RViz | '+{'NavFnDijkstra':'NavFn Dijkstra','SmacAStar':'Smac A-star'}[sys.argv[2]]+' | Dense 3D map'
rows=subprocess.check_output(['wmctrl','-l'],text=True).splitlines();ids=[next(x.split()[0] for x in rows if pat in x) for pat in patterns]
for wid,(x,y) in zip(ids,[(100,80),(2100,80),(100,1190),(2100,1190)]):
 subprocess.run(['wmctrl','-ir',wid,'-b','remove,maximized_vert,maximized_horz'],check=True)
 subprocess.run(['wmctrl','-ir',wid,'-e',f'0,{x},{y},1700,850'],check=True)
 subprocess.run(['wmctrl','-ir',wid,'-b','add,above'],check=True)
time.sleep(2)
root=subprocess.check_output(['xwininfo','-root'],text=True);sw=int(re.search(r'Width:\s+(\d+)',root)[1]);sh=int(re.search(r'Height:\s+(\d+)',root)[1]);rects=[]
for wid in ids:
 s=subprocess.check_output(['xwininfo','-id',wid],text=True)
 x=int(re.search(r'Absolute upper-left X:\s+(-?\d+)',s)[1]);y=int(re.search(r'Absolute upper-left Y:\s+(-?\d+)',s)[1]);w=int(re.search(r'Width:\s+(\d+)',s)[1]);h=int(re.search(r'Height:\s+(\d+)',s)[1])
 assert 0<=x<sw and 0<=y<sh and x+w<=sw and y+h<=sh,(x,y,w,h,sw,sh)
 assert h<=950,('Window panel layout requires repair before recording',wid,h)
 rects.append(dict(window=wid,x=x,y=y,width=w,height=h))
filters=['[0:v]split=4[p0][p1][p2][p3]']
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
for i,(rect,title) in enumerate(zip(rects,titles)):
 filters.append(f"[p{i}]crop={rect['width']}:{rect['height']}:{rect['x']}:{rect['y']},scale=960:504:force_original_aspect_ratio=decrease,pad=960:540:(ow-iw)/2:36+(504-ih)/2:color=0x0b1421,drawtext=fontfile={font}:text='{title}':x=18:y=9:fontsize=19:fontcolor=0xe7f1ff[q{i}]")
filters.append('[q0][q1][q2][q3]xstack=inputs=4:layout=0_0|960_0|0_540|960_540[out]')
cmd=['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab','-draw_mouse','0','-framerate','20','-video_size',f'{sw}x{sh}','-i',':0.0','-filter_complex',';'.join(filters),'-map','[out]','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-r','20','-t','650',str(R/'go2w_four_screen_demo.mp4')]
(R/'recording_manifest.json').write_text(json.dumps(dict(start_wall_time=time.time(),source='one X11 frame cropped into four simultaneous views',rects=rects,titles=titles,output_dimensions=[1920,1080],fps=20,command=cmd),indent=2))
(R/'record_command.sh').write_text(shlex.join(cmd)+'\n')
print('Starting synchronized four-screen capture',rects,flush=True)
with (R/'recording.log').open('w') as f:
 p=subprocess.Popen(cmd,stdout=f,stderr=f);(R/'ffmpeg.pid').write_text(str(p.pid));p.wait();print('Recorder exited',p.returncode,flush=True)
