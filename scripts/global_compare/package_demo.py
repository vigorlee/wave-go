"""Package two verified continuous takes without cutting either traversal."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dijkstra', type=Path, required=True)
    ap.add_argument('--astar', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    runs = [args.dijkstra.resolve(), args.astar.resolve()]
    reports = [json.loads((r/'verification.json').read_text()) for r in runs]
    for report, planner in zip(reports, ['NavFnDijkstra', 'SmacAStar']):
        assert report['planner'] == planner
        assert report['navigation_passed'] and report['all_12_in_order']
        assert report['video_full_decode_ok']
    video = args.output/'officeworld_two_global_planners_demo.mp4'
    assert not video.exists(), 'Preserve existing exports; choose a fresh directory.'
    concat = args.output/'continuous_takes.ffconcat'
    concat.write_text('ffconcat version 1.0\n'+''.join(
        "file '"+str(r/'go2w_four_screen_demo.mp4').replace("'", "'\\''")+"'\n" for r in runs))
    subprocess.run(['ffmpeg', '-v', 'error', '-f', 'concat', '-safe', '0', '-i', str(concat),
                    '-c', 'copy', '-movflags', '+faststart', str(video)], check=True)
    metadata = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-show_entries', 'format=duration:stream=codec_name,width,height,r_frame_rate',
        '-of', 'json', str(video)], text=True))
    expected = sum(float(r['video_metadata']['format']['duration']) for r in reports)
    assert abs(float(metadata['format']['duration'])-expected) < .15
    previews = []
    for run, report, prefix in zip(runs, reports, ['dijkstra', 'astar']):
        manifest = json.loads((run/'recording_manifest.json').read_text())
        events = [json.loads(s) for s in (run/'course/events.jsonl').read_text().splitlines()]
        picks = {
            'stairs': next(e['time'] for e in events if e['state']=='stage_starting' and e['stage_index']==8)+10,
            'goal': events[-1]['time']-1,
        }
        waits = [e for e in events if e['state']=='pedestrian_yield']
        if waits:
            picks['pedestrians'] = waits[0]['time']+.8
        for label, stamp in picks.items():
            seconds = max(0, stamp-manifest['start_wall_time'])
            dest = args.output/f'{prefix}_{label}.png'
            subprocess.run(['ffmpeg','-v','error','-ss',f'{seconds:.3f}','-i',str(run/'go2w_four_screen_demo.mp4'),
                            '-frames:v','1','-update','1',str(dest)],check=True)
            previews.append(dict(file=str(dest.resolve()), source=str(run/'go2w_four_screen_demo.mp4'),
                                 video_time_s=seconds, timing='Event time minus recorder start; frame clock rounding applies.'))
    summary = dict(video=str(video.resolve()), metadata=metadata, runs=[str(r) for r in runs],
                   concatenation='Complete Dijkstra traversal followed by complete A-star traversal; original takes unchanged.',
                   astar_segment_start_s=float(reports[0]['video_metadata']['format']['duration']),
                   reports=reports, previews=previews)
    (args.output/'demo_manifest.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps({'video':str(video.resolve()),'duration_s':expected,'previews':len(previews)}))


if __name__ == '__main__':
    main()
