# OLI HU_D04_01 stair policy

This package trains an OLI-specific blind stair policy. It does not use G1 or
H1 weights. The deployed actor keeps the official LimX interface:

- input: five 102-value proprioceptive frames (`510` total)
- actor: `510 -> 512 -> 256 -> 128 -> 31`, with ELU activations
- output: 31 OLI PR-space joint actions with a smooth `[-4, 4]` bound
- control: the `walk_param.yaml` action scales, default pose, gains, and torque
  limits

The final stage matches the YardWorld staircase: 10 steps, 0.20 m tread depth,
and 0.15 m rise.

## Environment

The project training environment is already located at `.venv-oli-train`. Run
all commands from the project root:

```bash
source .venv-oli-train/bin/activate
export PYTHONPATH="$PWD:$PWD/third_party/mujoco_playground"
```

Validate model generation, GPU MJX, the official actor import, and ONNX export:

```bash
python -m training.oli_stairs.smoke_test
```

## Training

Run the full flat -> 5 cm -> 10 cm -> 15 cm curriculum:

```bash
scripts/train_oli_stairs.sh
```

Training length and parallel environment count can be overridden without
editing the script:

```bash
OLI_FLAT_STEPS=5000000 \
OLI_H05_STEPS=20000000 \
OLI_H10_STEPS=30000000 \
OLI_H15_STEPS=50000000 \
OLI_TRAIN_ENVS=2048 \
scripts/train_oli_stairs.sh
```

The final deployment candidate is written to:

```text
artifacts/oli_stairs/curriculum/h15/policy.onnx
```

This file is intentionally separate from the official flat-ground policy. It
must pass the full-model MuJoCo rollout before the navigation controller is
allowed to select it.

## Evaluation

```bash
python -m training.oli_stairs.evaluate \
  --stage h15 \
  --checkpoint artifacts/oli_stairs/curriculum/h15/checkpoints \
  --video artifacts/oli_stairs/curriculum/h15/rollout.mp4
```
