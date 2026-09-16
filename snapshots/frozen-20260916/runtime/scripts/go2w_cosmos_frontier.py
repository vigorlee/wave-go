"""Actual local Cosmos3-Edge inference over online, measured frontier choices."""
import json, os, subprocess, time
from pathlib import Path
import re

ROOT = Path('/home/unitree/matrix_g1_lcm_demo')

def validate_choice(text, count):
    # Only consume the final answer, never a choice mentioned in reasoning.
    final = text.rsplit('</think>', 1)[-1].strip()
    if final.startswith('```'):
        final = re.sub(r'^```(?:json)?\s*', '', final)
        final = re.sub(r'\s*```$', '', final)
    value = json.loads(final)
    if isinstance(value, list) and len(value)==1:value=value[0]
    if not isinstance(value, dict):raise ValueError('Expected exactly one Cosmos decision')
    if value.get('action') == 'hold':
        return dict(action='hold', reason=str(value.get('reason', '')))
    i = value.get('candidate_id')
    confidence = value.get('confidence')
    if value.get('action') != 'navigate' or type(i) is not int or not 0 <= i < count:
        raise ValueError('Cosmos returned an invalid measured candidate ID')
    if type(confidence) not in (int, float) or not .6 <= confidence <= 1:
        raise ValueError('Cosmos confidence is insufficient')
    return value


def choose_frontier(image, candidates, pose, yaw, output):
    job = output / ('cosmos_' + str(time.time_ns()))
    job.mkdir(parents=True)
    camera = job / 'camera.jpg'; camera.write_bytes(image)
    prompt = '''You are Cosmos, the visual high-level navigator of a simulated Unitree Go2-W.
Task: navigate as far as possible ahead along the initial forward direction while building a map online.
There is NO prebuilt map or route catalog. Choices below are computed from live LiDAR free-space connectivity.
Choose the farthest promising safe candidate, preferring greater forward_progress_m, considering the RGB view.
Bearing is relative to the current robot heading in radians, positive left. The image faces forward.
Do not assume a candidate behind the camera is visible. Do not claim global farthest or task completion.
Reject visible people, impassable walls, and stairs requiring unvalidated terrain traversal.
The robot stops during reasoning. A live geometry check and Nav2 collision avoidance run after your choice.
Select EXACTLY ONE candidate, not a list of alternatives. Keep reasoning under 100 words and the final reason under 30 words. Output one JSON object only: {"action":"navigate","candidate_id":0,"confidence":0.8,"reason":"visible evidence and geometric context"}.
Use {"action":"hold","reason":"why"} if no choice is justified. Never invent coordinates or velocity.
Measured candidates:\n''' + json.dumps(candidates)
    request = job / 'request.json'
    request.write_text(json.dumps(dict(model_mode='reasoner', prompt=prompt, vision_path=str(camera), max_new_tokens=1536, do_sample=False, repetition_penalty=1.1), indent=2))
    framework = ROOT / 'packages/cosmos-framework'
    env = os.environ.copy()
    env.update(LD_LIBRARY_PATH='', PYTHONPATH='', PYTORCH_CUDA_ALLOC_CONF='expandable_segments:True', PYTORCH_ALLOC_CONF='expandable_segments:True', COSMOS_TRAINING='0')
    start = time.monotonic()
    with (job/'inference.log').open('w') as log:
        result = subprocess.run([str(framework/'.venv/bin/python'), '-m', 'cosmos_framework.scripts.inference', '--parallelism-preset=latency', '-i', str(request), '-o', str(job/'output'), '--checkpoint-path', str(ROOT/'Cosmos3-Edge'), '--config-file', str(framework/'cosmos_framework/inference/configs/model/Cosmos3-Edge.yaml'), '--seed', '0', '--no-guardrails'], cwd=framework, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=600)
    if result.returncode: raise RuntimeError(f'Cosmos inference failed; see {job}/inference.log')
    text = (job/'output/request/reasoner_text.txt').read_text()
    choice = validate_choice(text, len(candidates))
    choice.update(job=str(job), elapsed_sec=time.monotonic()-start)
    (job/'decision.json').write_text(json.dumps(choice, indent=2))
    return choice
