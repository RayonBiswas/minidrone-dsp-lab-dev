# CLAUDE.md: Mini-Drone DSP Lab (handoff context)

## Who and what
- Owner: Rayon Biswas, T.Y. B.Tech EXTC, VJTI Mumbai. Team: Zaid Faruqui, Tanisha Nevrekar, Piyush Patil, Ananya Rane, Arpit Rajput.
- Course: DSP assignment "Digital Signal Processing in Today's Era". The results presentation is due in the 2nd–3rd week of November 2026. The project is also entered in the MathWorks Minidrone Competition India 2026.
- Project: **Noise-Robust Path Tracking for an Autonomous Mini-Drone**. It uses classical DSP only (no AI/ML) to clean each noisy sensor so a mini-drone can follow a coloured track using a downward camera and IMU.
- The aim, from proposal section 4, is to prove the chain works: inject noise on purpose, measure with metrics, and compare filter vs filter.
  - Metrics: detection accuracy %, SNR improvement (dB), RMSE of angle and height, tracking error, processing time per frame.
  - Comparisons: filter vs filter, FIR vs IIR (phase delay), gyro-only vs accel-only vs complementary vs Kalman.

## Live links and branches
- Live app (GitHub Pages): https://rayonbiswas.github.io/minidrone-dsp-lab/
- Repo: https://github.com/RayonBiswas/minidrone-dsp-lab
- Branches:
  - `main`: development.
  - `gh-pages`: **what the live site serves**. After every change, push to both: `git push origin main && git push origin main:gh-pages`.
  - `checkpoint-v1`: frozen first working version. Do not touch it.

## Files
- `index.html`: the entire phone web app in one file (vanilla JS and canvas, no build step).
- `matlab/minidrone_dsp_sim.m`: full MATLAB simulation. Sections: vision stages, Monte Carlo, ablation (2b), IMU, altitude, closed loop, summary. Runs on plain MATLAB (toolbox functions written out as local functions); verified end-to-end on R2026b. Roll metrics skip a 1 s filter warm-up.
- `matlab/phone_sensor_demo.m`: MATLAB Mobile version. It reads the phone accel/gyro (`mobiledev`), runs FFT → notch → LPF → fusion, then takes a camera snapshot and runs the vision path. Also unverified.
- MATLAB style Rayon prefers: self-contained runnable `.m` scripts with local functions at the end, minimal student-style comments, and explicit per-step blocks rather than clever abstractions.

## App tabs (index.html)
- **Camera**: synthetic track, live phone camera, or photo. Pipeline: Median 3×3 (on the image) → optional Gaussian 5×5 → RGB→HSV threshold → close + open (disk r=2) → [bottom-½ ROI] → [biggest blob] → boundary edges → Hough (top 6 peaks, same-angle group, ρ = midpoint of the two track edges) → [least-squares refine when blob is on] → steering low-pass → PD steer.
  - Real-camera fixes: tap the track in panel 1 to pick its colour (mean of the 5 most saturated pixels in a 5×5 patch → hue ±0.04, sat ≥ 0.6·s, val ≥ 0.45·v; 'Picked' button). On camera/photo, morphology becomes a 3×3 close only (any opening erases a 2 px cable) and the ROI chip turns on. On synthetic, the ROI is off by default. The Results tab (E1–E5) does not use these stages, so its seeded numbers are unchanged.
  - `test_images/real_floor.png` is the regression frame: true 7.1° / 0 px; before the fix −12°, after 7.1° / 0.0 px.
  - Dashed pipeline chips toggle the filters. Sliders inject Gaussian noise, per-channel salt & pepper (like MATLAB `imnoise`), uneven lighting and motion blur.
  - The yellow line is the raw pipeline, green is DSP, white dashed is ground truth (synthetic only).
- **IMU**: live DeviceMotion (accelerationIncludingGravity + rotationRate) with a simulated fallback.
  - Chain: FFT on |a| finds up to 2 vibration peaks → auto notch biquads → IIR Butterworth or FIR (Hamming windowed-sinc) low-pass → roll = atan2(ax, hypot(ay, az)).
  - Gyro sign is auto-calibrated by correlation.
  - Fusion: gyro-only, complementary (α slider) and Kalman (angle + bias).
  - Readouts: fs, vibration peak, noise, drift, LPF delay at 1 Hz.
- **Altitude**: simulated take-off/hover/landing; median 7 → moving average 15 → 1D Kalman; spike burst button.
- **Flight**: a competition-style straight-segment track. Two drones get the same noise: raw (yellow) vs DSP (blue). Shows tracking error and steering jerk.
- **Drone**: camera + IMU together. Phone held flat, back camera down, top edge = nose. Artificial horizon shows roll and pitch, plus tracking and steering.
- **Navigate** (main localisation demo): optical flow by FFT phase correlation on 128×128 grayscale patches (Hann window, quality gate q<0.04, median of 3, EMA low-pass), gyro heading (rotationRate.alpha) with 1 s bias calibration at Set start and compass fusion (deviceorientationabsolute, 0.02 gain). The Calibrate 50 cm button learns cm/px and the camera axis, saved in localStorage. Arena 150×100 cm, obstacle, A* on a 5 cm grid with line-of-sight smoothing, re-planned every 300 ms; arrow + speechSynthesis directions; 'I'm at the goal' measures final error. Sim: value-noise floor at 5.12 px/cm with ground truth. E6 drift test: mean 11 cm goal error (7% drift over ~154 cm) filtered vs ~86 cm raw. Unit test: phase correlation recovers known shifts to within 0.03 cm.
- **Navigate + ESP32** (current localisation plan): sources 'Simulate ESP32' / 'ESP32 via Bluetooth'. Room presets 1.5×1 m / 5×4 m + custom W×H, tap-to-edit obstacles, optional 4th anchor. ESP chain = processRSSI (median 5 → RSSI Kalman → path-loss → linear trilateration → weighted LS, w = 1/(0.05+(0.25d)²) → position Kalman); arrival radius 50 cm held 10 steps; lookahead 40 cm; instruction hold 5 steps; re-plan every 4 direction calls. Sim (5×4 m, 8 runs): goal error median 135 cm (no WLS) → 55 cm (WLS) → 39 cm (WLS + hold); 4 anchors gives the best tracking (~54 cm RMS vs ~2.3 m raw). Firmware: hardware/esp32_beacon (NUM_ANCHORS 3/4).
- **Beacon page** (no nav button, opened from Navigate): ESP32 filter switches, calibration fit, and the standalone beacon sim. Sim with path-loss + shadowing + multipath fades; raw vs filtered drones fly start → goal; calibration fit (P0, n); live input via Web Bluetooth (Nordic UART, device DRONE-RX) or Web Serial 115200; CSV replay (hardware/esp32_beacon/sample_log.csv). Sim result: position error about 2.2 m raw → about 0.8–1.0 m filtered; the median gives the biggest gain; the position Kalman mainly smooths the path rather than cutting RMS error.
- **Results**: E1–E5 with a fixed seed (mulberry32 `setSeed`). Live sensor sims use `randnM` (Math.random) so they never disturb the seeded runs. This includes the altitude sim `altTick()`, which runs every frame on every tab: when it used the seeded `randn()`, E5 changed on every run (fixed). Copy tables (Markdown) and Download CSV.

## Current results (fixed seed, deterministic)
- **E1 vision, filter vs filter:**
  - Salt & pepper: Median + morph (ours) wins, 100% success / 0.31° vs raw 90% / 1.84°. Median beats Gaussian (0.31° vs 0.47°).
  - Gaussian noise: Median 0.51°, Gaussian 0.63°, raw 2.54°.
  - Mixed noise: the Gaussian alone is best and cheapest (0.62°, 2.7 ms) vs ours 0.71° (11.9 ms), which is within the spread.
  - Morphology alone hurts.
- **E2 noise sweep:** raw angle error goes from 0.5° to 3.2° as salt & pepper rises to 25%. All filtered versions stay ≤ 0.54°.
- **E3 inertial:**
  - IIR LPF 82 ms delay vs FIR (41 taps) 200 ms; IIR 1.8° vs FIR 4.5° roll RMSE.
  - The notch adds nothing at 18/36 Hz (the LPF already removes it), but at 5 Hz vibration it cuts error from 5.9° to 1.9°.
  - Gyro-only 21° (drift), complementary 1.4°, Kalman 0.7°.
- **E4 altitude:** raw 20 dB → median + Kalman 34.6 dB SNR. Linear filters smear spikes; the median removes them.
- **E5 flight:** both drones stay on track. The DSP drone steers 8.0× more smoothly (jerk 0.847 → 0.106) at about 1 cm more tracking error (2.15 vs 3.41 cm RMSE, low-pass lag). Same numbers every run.
- **Honesty rule:** never tune or fake results to make "ours" win. Hough voting is already robust, so the DSP gain is precision and smoothness. Present trade-offs openly.

## Commit rules
- Commits authored as RayonBiswas only. No Co-Authored-By or Claude-Session lines, no "Generated with" footers.
- No force-push or history rewrite without his go-ahead and a backup.

## Known issues
- **Brave browser blocks motion sensors**; use Chrome on the phone (Android). In-app browsers (WhatsApp/Gmail) also block sensors.
- When sensors fail, the IMU tab shows a "Diagnostics:" line (secure page, DeviceMotionEvent present, events received).
- Phone IMU samples at about 60–100 Hz, so vibration injection is capped at 0.45·fs.
- Inside the claude.ai artifact viewer, camera, motion and downloads are blocked (simulated mode only). Use the GitHub Pages link.

## Current task: record the demo on the laptop
- Goal: a roughly 3-minute video showing the **phone screen** and the **phone/hands moving over the red cable** at the same time, so it is clearly live, not hardcoded.
- Setup on Windows (Mac is similar):
  1. **scrcpy** (github.com/Genymobile/scrcpy releases). On the phone, enable Developer options → USB debugging, connect USB, then run `scrcpy` or `scrcpy --record=demo.mp4`.
  2. **OBS Studio** sources: Window Capture (scrcpy window) as the main view, Video Capture Device (laptop webcam on the hands/cable) in a corner, and Mic. Set the recording format to mp4.
  3. Quick fallback: the phone's built-in Screen recorder with show-taps and mic.
- Shot list:
  1. Camera tab, synthetic track: raise salt & pepper, toggle Median off/on.
  2. Camera tab, phone camera on the red cable: rotate and slide the phone so θ, d and the steering respond.
  3. IMU tab: tilt; shake sideways without tilting (raw jumps, fused stays steady); fan → FFT peak → notch, then Auto notch off; switch IIR → FIR to show the delay.
  4. Drone tab: walk along the cable holding the phone flat.
  5. Flight tab: raw vs DSP steering plot.
  6. Results tab on the laptop: tables and verdicts.

## Plan A (current direction, see PLAN_A.md)
- Stick to the proposal: track following + landing marker, not map/navigation (Navigate tab stays as an extra, not the core).
- P1: Simulink minidrone with `matlab/simulink/` blocks (dspVisionBlock, addCameraNoise, steeringLowpass). Untested in Simulink.
- P2: ESP32 + MPU6050 + unbalanced DC motor, firmware `hardware/esp32_mpu6050/` at 500 Hz, internal DLPF off; IMU tab 'ESP32 + MPU6050 (USB)' via Web Serial 921600 baud, 'Record 20 s CSV'; analysis `matlab/esp32_mpu6050_analysis.m`. Phone IMU (60–100 Hz) aliases motor vibration, hence the rig.
- P3: real red-tape track + blue landing pad; Camera tab pad detection (median + morph blue mask, 6-of-8-frame vote → LAND ✓, flicker counter).

## Demo plan (Navigate, extra)
- Arena: chart paper or newspaper about 1.5 × 1 m with START, GOAL and a book as the obstacle.
- Steps: Calibrate 50 cm once → Set start (hold still 1 s) → follow the arrow and voice → go the wrong way on purpose (it re-plans) → reach the goal, tap 'I'm at the goal' to get the error → switch off Hann window or bias cal and repeat to show the raw drift.

## Next tasks (in order)
1. Record the demo video (above).
2. Make the presentation slides: problem → pipeline → results tables/graphs → live demo → limitations → next steps.
3. Run `matlab/minidrone_dsp_sim.m` in MATLAB and fix any errors.
4. Later: plug into the Simulink minidrone model (swap the synthetic frames for the model's camera output), and optional ESP32 + MPU6050 hardware (about ₹1,700).

## Viva answers to keep handy
- Median vs Gaussian: the median discards impulses; the Gaussian averages them into neighbours.
- IIR vs FIR: IIR has lower delay, which matters inside a control loop; FIR is linear-phase but slow.
- Complementary vs Kalman: Kalman is more accurate; complementary is nearly as good for far less compute.
- Yaw: the accelerometer cannot sense it because gravity doesn't change when you turn flat, so heading must come from the gyro (drifts) or the camera line angle θ.
- Why the raw drone survives: Hough voting is robust; the DSP adds precision and smooth commands, which stop motor oscillation on a real drone.

## Phone → Laptop link
- Header button 📡 opens `#linkCard`. Laptop gets a 6-digit code (PeerJS id `mdsp-lab-<code>`); phone connects with it (`?link=<code>` pre-fills it).
- Phone sends its back camera as a WebRTC call and each devicemotion / deviceorientationabsolute event over a JSON data channel (`['m', ax, ay, az, alpha, beta, gamma, dt]`, `['o', alpha]`), with its own dt.
- Laptop re-dispatches them as window events (`e.linkDt` carries the phone's dt; `onMotion` and the Navigate listener use it), and `getCam()` returns the phone's stream, so the tabs run unchanged. `linkOwns()` stops a linked stream from being stopped when switching sources.
- PeerJS 1.5.5 is loaded from unpkg only when the link is used. `?peerhost=&peerport=&peerpath=&peersecure=1` points it at your own PeerJS server.
- Tested with two headless Chromium pages, a local PeerJS server and a fake camera/motion source (camera 640x480 on the laptop, IMU live with the phone's 20 deg tilt, Drone LIVE camera + LIVE IMU, stop/reconnect).
- Navigate mirroring: the phone sends `{k:'nav', ...}` 10×/s (pos, psi, dirs, remaining route; every 10th message also map: w, h, grid, inflate, room, start, goal, obstacles, src, anchors, and thinned paths). The laptop sets `NV.mirror`, which stops its own nav stepping and gyro heading, and draws the phone's state. `linkEndMirror()` on disconnect.
- Route export (`#nRouteCsv`): A* START→GOAL, cut to corners with `clearLine()` (string pulling, keeps the obstacle margin). CSV: i, x_cm, y_cm, x_fwd_m (= Δy), y_right_m (= Δx). Phone and laptop export the same file because the map, grid and margin are mirrored.

## Navigate voice
- `directions()` gives short classes with hysteresis: Go straight (<15/30°), Slight left/right, Turn left/right (>50/60°), Turn around (>125/135°); degrees go in the sub-line, not the speech.
- `holdDirs()` keeps the display live and marks `d.stable` once the class has held 250 ms (600 ms with ESP32). `speak()` only speaks stable instructions, cancels a stale sentence the moment the class changes, repeats turns every 4 s and straight every 8 s, says Arrived once, and prefixes the first one with "Go.".
- Start: green "face this way" arrow at START (heading 0 = up the map). Tap Set start, hold still ~1 s (gyro bias) until "Go".

## Navigate: Follow track (camera)
- Source `line`: `lineStep()` runs `pipeline()` on each camera frame (Camera tab colour `vCol`, `vThr`, ROI chip; median + thin close + biggest blob always on). Steering angle `sa = th + atan(d / 60 px)` (+ = right), low-passed 0.6/0.4, classes with hysteresis: straight < 14/8°, slight, turn > 40/32°.
- No track for 600 ms -> "Track lost"; blue pad in 6 of 8 frames -> "Landing pad: stop" (arrived). `lineView()` draws the frame + detected line in place of the map; the laptop mirror draws the phone's estimate over its copy of the phone's video.
- Tested with a canvas-stream fake camera: straight, slight/turn right and left, offset, lost, pad all give the right instruction; voice matches the screen.

## Maze red path (Navigate, `#mzCard`)
- `mzLoad()` scales the photo to ≤480 px; `mzProcess()`: red = r > 0.3 and r > 1.5 g, b; close 3×3; largest 8-connected blob; Zhang-Suen; longest path by two BFS (ignores spurs); start = end nearer the top-left (Swap button). Tilt = average of 4θ over the path (corridors repeat every 90°); `MZ.toM/toP` rotate between photo and maze axes.
- `mzCorners()`: Douglas-Peucker (eps 1.2% of size); Square corners squares legs within 20° of the maze axes and drops straight corners (2 passes). Scale = typed width / wall box (dark pixels, 1st-99th percentile in maze axes).
- CSV like the route export (`drone_route_maze.csv`, 3 comment lines + header). Tested: `test_images/maze_red_path.png` → 30 points, 28 turns, 4.42 m at 1.5 m; a 6° rotated, blurred, noisy, unevenly lit copy → same 28 turns, 4.32 m.
- Maze finding (2026-11-18b): Otsu on luminance; the paper = biggest bright 4-connected region; walls = darker than 0.72 × local mean (integral image) and mostly surrounded by paper (so the paper's edge and the background are not walls). Tilt = angle with the sharpest row/column projections of the walls. Box = outermost rows/columns holding > 30% of the longest wall line.
- Red is only used if it is saturated red (r > 2 g, r > 1.8 b, r - g > 70), inside the maze box and its traced length > 0.6 × maze side; otherwise `mzSolve()`.
- `mzSolve()`: each outer wall fitted as its own line (perspective), gaps = stretches without wall pixels within a band, at least 1.2 × the 90th-percentile wall distance (~ a corridor wide); the two longest, start = nearer top-left. Free space = inside the four lines, walls thickened 1 px. Dijkstra with 4-neighbour moves, cost 1 + 40/d² (d = distance to a wall) so the route runs down corridor middles. Taps on the canvas (S, then E) override the gaps.
- Tested: user's photo of a maze on a phone screen (slanted, with keyboard, hand, wood) → entrance bottom-left, exit right, 0 wall crossings; maze_no_path.png → same turns as the red solution.
