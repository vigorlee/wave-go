from pathlib import Path
import json,subprocess,cv2
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent
m=json.loads((R/'recording_manifest.json').read_text());events=[json.loads(x) for x in (R/'course/events.jsonl').read_text().splitlines()]
result=json.loads((R/'course/result.json').read_text());assert result['passed'],result
nav=next(e['time'] for e in events if e['state']=='navigating');end=next(e['time'] for e in events if e['state']=='succeeded')
cut=max(0,nav-m['start_wall_time']-3);length=end-m['start_wall_time']-cut+5
raw=R/'go2w_four_screen_raw.mp4';out=R/'go2w_four_screen_demo.mp4'
if not raw.exists():out.rename(raw)
cmd=['ffmpeg','-v','error','-y','-ss',str(cut),'-i',str(raw),'-t',str(length),'-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart','-an',str(out)]
subprocess.run(cmd,check=True)
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(out)]));stream=probe['streams'][0];duration=float(probe['format']['duration']);assert (stream['width'],stream['height'])==(1920,1080);assert stream['avg_frame_rate']=='20/1'
checks=[('Start',1.)]
for name,idx in [('Stair ascent',0),('Stair descent',1),('Obstacles',4),('Ramp',8)]:
 e=next(e for e in events if e['state']=='stage_completed' and e['stage_index']==idx)
 checks.append((name,max(1.,min(duration-1,e['time']-m['start_wall_time']-cut-2))))
checks.append(('Goal reached',duration-2))
cap=cv2.VideoCapture(str(out));sheet=Image.new('RGB',(1280,1140),'#0b1421');font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',19);samples=[]
for i,(label,t) in enumerate(checks):
 cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read();assert ok
 path=R/f'frame_{i+1:02d}.png';cv2.imwrite(str(path),frame);im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));im.thumbnail((640,360),Image.LANCZOS)
 x=(i%2)*640;y=(i//2)*380;sheet.paste(im,(x,y));ImageDraw.Draw(sheet).text((x+8,y+359),f'{label} | {t:05.1f} s',font=font,fill='white')
 samples.append(dict(label=label,video_seconds=t,path=str(path)))
cap.release();sheet.save(R/'video_contact_sheet.png')
decode=subprocess.run(['ffmpeg','-v','error','-i',str(out),'-f','null','-'],capture_output=True,text=True);assert decode.returncode==0,decode.stderr
report=dict(video=str(out),width=1920,height=1080,fps=20,duration_seconds=duration,bytes=out.stat().st_size,trim_leading_seconds=cut,raw_video=str(raw),passed_stages=result['completed_stages'],full_decode_errors=decode.stderr,samples=samples,source='Single X11 capture; four simultaneous application crops, no interpolated or reconstructed sensor imagery',clock_note='Event offsets use wall time at recorder launch; approximate subsecond correspondence')
(R/'video_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
