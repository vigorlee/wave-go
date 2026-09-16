from pathlib import Path
import json,subprocess,cv2
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;m=json.loads((R/'source_manifest.json').read_text());manifest=json.loads((R/'replay_frames.json').read_text());duration=m['segment_duration'];cut=m['segment_wall_start']-m['original_video_wall_start'];out=R/'go2w_four_screen_demo.mp4'
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
filters=f"[0:v]setpts=PTS-STARTPTS[base];[1:v]scale=960:504:force_original_aspect_ratio=decrease,pad=960:540:(ow-iw)/2:36+(504-ih)/2:color=0x0b1421,drawtext=fontfile={font}:text='02  RViz | Same-run LiDAR replay + inflation':x=18:y=9:fontsize=18:fontcolor=0xe7f1ff,fps=20[pcd];[base][pcd]overlay=x=960:y=0:shortest=1[out]"
cmd=['ffmpeg','-v','error','-y','-ss',str(cut),'-i',m['original_video'],'-framerate','1','-i',str(R/'rviz_frames/%04d.png'),'-filter_complex',filters,'-map','[out]','-t',str(duration),'-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-r','20','-movflags','+faststart','-an',str(out)]
(R/'assembly_command.json').write_text(json.dumps(cmd,indent=2));subprocess.run(cmd,check=True)
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(out)]));stream=probe['streams'][0];duration=float(probe['format']['duration']);assert (stream['width'],stream['height'])==(1920,1080);assert stream['avg_frame_rate']=='20/1'
events=[json.loads(x) for x in (Path(m['source_run'])/'course/events.jsonl').read_text().splitlines()];checks=[('Start',1.)]
for name,idx in [('Stair ascent',0),('Stair descent',1),('Obstacles',4),('Ramp',8)]:
 e=next(e for e in events if e['state']=='stage_completed' and e['stage_index']==idx);checks.append((name,max(1.,min(duration-1,e['time']-m['segment_wall_start']-2))))
checks.append(('Goal reached',duration-2));cap=cv2.VideoCapture(str(out));sheet=Image.new('RGB',(1280,1170),'#0b1421');font=ImageFont.truetype(font,18);samples=[]
for i,(label,t) in enumerate(checks):
 cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read();assert ok;path=R/f'frame_{i+1:02d}.png';cv2.imwrite(str(path),frame);im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));im.thumbnail((640,360),Image.LANCZOS);x=(i%2)*640;y=(i//2)*390;sheet.paste(im,(x,y));ImageDraw.Draw(sheet).text((x+8,y+363),f'{label} | {t:05.1f} s',font=font,fill='white');samples.append(dict(label=label,video_seconds=t,path=str(path)))
cap.release();sheet.save(R/'video_contact_sheet.png');decode=subprocess.run(['ffmpeg','-v','error','-i',str(out),'-f','null','-'],capture_output=True,text=True);assert decode.returncode==0,decode.stderr
report=dict(video=str(out),width=1920,height=1080,fps=20,duration_seconds=duration,bytes=out.stat().st_size,source_successful_stages=11,full_decode_errors=decode.stderr,samples=samples,rviz_replay_updates_hz=1,rviz_point_voxel_m=.04,final_accumulated_points=manifest[-1]['accumulated_voxels'],pointcloud_timestamp_alignment='source sensor stamp <= displayed timeline instant; no future scans',accuracy_note='The recording wall start has subsecond launch uncertainty; pointcloud snapshots update once per second, while the three original panels retain original 20 fps screen video.')
(R/'video_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
