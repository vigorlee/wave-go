# Figure 1 | WAVE-Go: world-model action adaptation with verified execution

**a, Perceive and generate.** A pretrained vision-action world model
(instantiated with Cosmos3-Edge) consumes the live first-person RGB observation
and language task. Its Reasoner emits a strict six-field semantic decision,
whereas its Generator produces a 16-step, 9-dimensional egocentric
relative-pose action chunk. The backbone is used without Go2-W-specific
fine-tuning.

**b, Adapt and execute.** A training-free geometric interface projects the
Generator's 6D rotation representation onto \(SO(3)\), transforms camera-frame
relative poses into the Go2-W base frame, and extracts bounded planar
\((v_x,\omega_z)\) commands. Seeded candidates are selected from model outputs,
and a clearance-dependent action prefix (\(K=16,12,8\)) is executed at 10 Hz.
RGB-D, LiDAR and odometry are revalidated before every step by a veto-only
safety shield. The shield may reduce or cancel an action but cannot create a
replacement trajectory. DreamWaQ tracks the resulting velocity command.

**c, Verify before completion.** Evidence sources have deliberately separated
authority. The Reasoner may authorize the transition from search to approach,
and short-term tracking may maintain continuity, but neither can authorize task
completion. Charging requires a currently exact ID 560 observation, synchronized
RGB-D range of 0.20-0.40 m, at least 0.42 m LiDAR clearance, and a stable,
upright robot stopped for approximately 3 s. Missing or stale evidence causes
the system to fail closed.

**d, Closed-loop demonstration.** Unmodified HouseWorld observations from the
successful run show map-independent search, exact marker and dock verification,
world-model-controlled approach, three consecutive close-range confirmations,
and stopping before the robot enters its simulated charge posture. In the
representative run shown here, the final RGB-D range was 0.357 m and the
crouched body height was approximately 0.222 m.

## One-sentence paper summary

WAVE-Go transfers a pretrained vision-action world model to a wheeled
quadruped through training-free geometric action adaptation, risk-adaptive
receding-horizon execution, and authority-separated multimodal verification.

## Artwork notes

- The SVG and PDF retain vector text, arrows, boxes and symbols.
- The experimental RGB panels are embedded raster observations and have not
  been synthesized or retouched.
- The 300 dpi PNG is suitable for review and slides.
- The 600 dpi PNG or PDF is recommended for final submission.
- All labels use a color-blind-accessible palette and remain distinguishable
  through text and position when printed in grayscale.
