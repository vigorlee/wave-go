"""Export evidence only after the recorded continuous mission actually passed."""
from pathlib import Path
import json,sys,subprocess,shutil,hashlib
from PIL import Image,ImageDraw,ImageFont
run=Path(sys.argv[1]).resolve();out=run.parent/(run.name+'_final');out.mkdir(exist_ok=True)
r=json.loads((run/'course/result.json').read_text());assert r['passed'] and r['completed_stages']==r['total_stages'],r
video=run/'go2w_four_screen_demo.mp4'
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(video)]))
subprocess.run(['ffmpeg','-v','error','-i',str(video),'-f','null','-'],check=True)
shutil.copy2(video,out/'officeworld_inline_second_floor.mp4')
start=json.loads((run/'recording_manifest.json').read_text())['start_wall_time'];duration=float(probe['format']['duration'])
events=[json.loads(s) for s in (run/'course/events.jsonl').read_text().splitlines()]
large=any(e.get('stage_id')=='large_stair_crest' for e in events)
choices=[('START',None),('LARGE STAIRS' if large else 'SMALL STAIRS','large_stair_crest' if large else 'small_stair_crest'),('RAMP','ramp_crest'),('CYLINDERS','pedestrian_crossing' if large else 'cylinder_corridor'),('LOWER FLIGHT','native_lower_landing'),('HALF LANDING','native_half_landing_turn'),('UPPER FLIGHT','native_upper_exit'),('SECOND FLOOR','second_floor_goal')]
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22);sheet=Image.new('RGB',(1920,4*580),'#0b1421');times=[]
for i,(label,stage) in enumerate(choices):
 st=next((e['time'] for e in events if e.get('stage_id')==stage and e['state']=='stage_starting'),start+2)
 en=next((e['time'] for e in events if e.get('stage_id')==stage and e['state']=='stage_completed'),st+2)
 t=2 if stage is None else ((st+en)/2-start)
 if stage=='second_floor_goal':t=duration-1
 t=max(.1,min(duration-.1,t));name=f'{i+1:02d}_{label.lower().replace(" ","_")}.png';p=out/name
 subprocess.run(['ffmpeg','-v','error','-ss',str(t),'-i',str(video),'-frames:v','1',str(p)],check=True)
 im=Image.open(p).convert('RGB');im.thumbnail((960,540));x=(i%2)*960;y=(i//2)*580;sheet.paste(im,(x,y));ImageDraw.Draw(sheet).text((x+16,y+546),f'{label}   |   {t:.1f} s',font=font,fill='#e7f1ff');times.append(dict(label=label,video_time_sec=t,file=str(p)))
sheet.save(out/'contact_sheet.jpg',quality=94)
for f in ['course/result.json','recording_manifest.json','ue_actor_validation.json']:shutil.copy2(run/f,out/Path(f).name)
report=dict(source_run=str(run),video=str(out/'officeworld_inline_second_floor.mp4'),duration_sec=duration,dimensions=[probe['streams'][0]['width'],probe['streams'][0]['height']],frames=probe['streams'][0].get('nb_frames'),full_video_decode='passed',mission=r,keyframes=times,video_sha256=hashlib.sha256(video.read_bytes()).hexdigest())
(out/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
