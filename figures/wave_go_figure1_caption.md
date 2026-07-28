# Figure 1 | WAVE-Go adapts world-model action chunks to verified wheel-legged execution

**a, Perception and generation.** A pretrained vision-action world model,
instantiated with Cosmos3-Edge without Go2-W-specific fine-tuning, receives the
current egocentric RGB observation, language task and stage context. Its
Generator is the sole source of nominal high-level motion and emits 16-step,
9-dimensional AV-domain relative-pose chunks. The separately invoked Reasoner
returns target semantics only after an exact marker candidate and stable-stop
precheck.

**b, Cross-embodiment execution.** Stage-adaptive seeded AV chunks are projected
onto \(SO(3)\), transformed from the optical-camera frame to the robot base
frame, and converted into bounded planar \((v_x,\omega_z)\) commands. A preview
veto and score select among model-generated candidates. The selected chunk is
executed with a clearance-dependent prefix: \(K=16,12,8\) during search and
\(K\leq8\) during approach, hold and reacquisition. RGB-D, LiDAR, odometry and
attitude are revalidated before every 0.1-s action step by a veto-only shield
that may keep, limit or cancel motion but cannot synthesize a replacement
trajectory. DreamWaQ tracks the surviving command with an effective forward
speed limit of \(0.35\,\mathrm{m\,s^{-1}}\) and a \(0.20\)-s command
time-to-live.

**c, Authority-separated completion.** Semantic confirmation authorizes the
transition into approach but has no direct completion authority. Arrival
requires current exact ID 560 evidence, synchronized RGB-D range, collision
clearance, odometry, attitude, alignment, approach travel and three new
synchronized frame pairs. The RGB-D arrival interval is 0.20-0.40 m; the
LiDAR threshold of at least 0.42 m measures forward collision clearance rather
than dock range. Once semantic permission and the physical arrival gates agree,
the robot stops for approximately 3 s while the evidence is continuously
rechecked. Only this stationary revalidation authorizes entry into the
simulated charge posture. Missing, stale or inconsistent evidence causes the
system to fail closed.

**d, Representative closed-loop run.** Frames from one successful HouseWorld
run show world-model exploration, exact ID 560 detection, semantic confirmation
with confidence 0.90 and the close-range evidence gate. The controller log
records that the robot then stopped before crouching. The final RGB-D range was
0.357 m, three consecutive exact close confirmations passed, and the verified
crouched body height was approximately 0.222 m with linear and angular speed
near zero.

## Production notes

- The artwork is designed natively at `183 x 132 mm`, not scaled down from a
  presentation canvas.
- Labels, arrows and boxes remain vector objects in the SVG and PDF.
- Experimental panels are crops of the original run observations; they have
  not been synthesized or retouched. The green ID pointer is an explanatory
  vector annotation rather than a detector bounding box.
- Use the PDF for submission when accepted by the journal workflow; otherwise
  use the 600 dpi PNG.
