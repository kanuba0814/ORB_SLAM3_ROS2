# H30 Quickstart: gentle-init → stable SLAM

> Target hardware: Yesense H30/H30WP/H30MINI IMU + 640×480 fisheye mono camera,
> pre-synchronised by `cam_imu_sync` into `/sync/image_raw` (30 Hz) and
> `/sync/imu` (~229 Hz).
> Kalibr extrinsic: `imu_cam_kalibr_E52_mono` (locked into yaml).

This file is the runbook you read **before each session**. It is short by
design. The why is in commits `c884187`, `091439a` and in
`extra_ws/debug_runs/fix_until_stable/200_root_cause_correction_and_deprecation.md`.

## TL;DR

ORB-SLAM3 mono-inertial cannot bootstrap robustly under fast rotation. You
must **walk gently for the first ~30 seconds**, then it is safe to move
normally. If you skip this, post-init IMU integration explodes (verified: pose
drifts to ~15 km on the H30 within 10 seconds of a bad init).

## Launch (live)

Two terminals inside the `autoware_env_v2_host` container.

```bash
# Terminal A — start cam_imu_sync (your sensor pipeline)
source /opt/ros/humble/setup.bash
source /home/gxxl/slam_ws_v2/driver_ws/install/setup.bash
ros2 launch cam_imu_sync cam_imu_sync.launch.py
```

```bash
# Terminal B — start ORB-SLAM3 wrapper
source /opt/ros/humble/setup.bash
source /home/gxxl/slam_ws_v2/driver_ws/install/setup.bash
source /home/gxxl/slam_ws_v2/extra_ws/install/setup.bash
ros2 launch orb_slam3_ros2 h30_sync_slam.launch.py debug_enable:=true
```

`debug_enable:=true` is recommended on the first runs; it prints the
`[orb_event] event_code=IMU_INIT_SUCCESS` line so you can confirm
initialisation finished. Once you trust the setup, drop it.

## Motion script for the first 30 seconds

| t (s) | What to do | Why |
|---|---|---|
| 0 – 5  | **Slow translation** (e.g. walk 0.5 m/s, hold camera level). Avoid any rotation. | Gives ORB the parallax it needs for monocular initialisation (`Initializer` requires `median_parallax ≥ 1.0 px`). |
| 5 – 15 | Keep translating slowly. Add **small** roll/pitch (< 15 °/s). Avoid yaw spins. | Lets `LocalMapping::InitializeIMU` average bias over diverse but mild KF orientations. The 8 s / 15 KF window we set in yaml needs this. |
| 15 – 30 | Slightly more motion is fine. Still no fast yaw, no shake. | Lets VIBA1 (mTinit > 5 s) and VIBA2 (mTinit > 15 s) refine gravity, scale, and bias. |
| 30 +   | **Normal use.** Whatever your application does. | Bias has converged; ORB-SLAM3 IMU integration is now stable. |

> Hard rule: in the first 30 s, keep `|gyro| < 18 deg/s`. Bursts of 25–65 deg/s
> during this window are what ORB absorbs as bias error, and the post-init
> integration then drifts catastrophically — this is a known limitation of
> vanilla ORB-SLAM3 IMU-mono initialisation (issues
> [#69](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/69),
> [#910](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/910),
> [#968](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/968) on the upstream).

## How to know init succeeded

With `debug_enable:=true` you will see, in order:

1. Several `[orb_event] event_code=MONO_INIT_FAILED_*` lines (normal, until
   parallax accumulates).
2. `[orb_event] event_code=IMU_INIT_SUCCESS frame_id=... estimated_scale=... bias_norm=...`
   — this is the success signal.
3. `start VIBA 1` / `end VIBA 1` about 5 s later.
4. `start VIBA 2` / `end VIBA 2` about 15 s later (only fires if you keep
   moving enough to insert keyframes; absent ≠ broken, just less refined).

Healthy values at `IMU_INIT_SUCCESS`:

- `bias_norm < 0.05` (slowMove validation had 0.019; truth is ~0.0005)
- `estimated_scale` between roughly 0.05 and 5 (it is monocular unit; absolute
  value doesn't matter, only that it is finite and non-NaN)
- `gravity_norm ≈ 9.81`

If `bias_norm > 0.1` (= 5.7 deg/s) at init, the run is already in the bad
regime. Restart the wrapper and rewind your motion.

## How to know SLAM is healthy mid-session

In RViz (`use_rviz:=true` is the default) you should see:

- `/orb_odom` arriving at 10 Hz, pose values changing smoothly (no jumps).
- `/live_point_cloud` accumulating points along your trajectory.
- No "IMU is not or recently initialized. Reseting active map..." prints
  after the first `IMU_INIT_SUCCESS`. **A single post-init reset means scale
  diverged — you have to restart and re-init.**

A useful one-liner during a live session:

```bash
grep -c "Reseting active map" /path/to/wrapper.log  # post-init this must stay at 0
```

## Validated configuration (don't change without re-validating)

In `config/Monocular-Inertial/rmoncam_h30.yaml`:

| Field | Value | Why |
|---|---|---|
| `Camera.fps` | 30 | matches live `/sync/image_raw`; drives ORB KF-insertion timing (`mMaxFrames = fps`) |
| `IMU.Frequency` | 229.0 | informational only (logging); pre-integration uses each sample's real timestamp |
| `LocalMapping.ImuMonoInitMinTime` | **8.0** | longer window → fewer high-motion KFs polluting bias (`091439a`) |
| `LocalMapping.ImuMonoInitMinKFs`  | **15** | same goal as above |
| `LocalMapping.ResetOnSmallMotionBeforeInertialBA2` | 0 | disables the small-motion reset trap |
| `Tracking.ResetActiveMapBeforeInertialBA2` | 0 | disables pre-BA2 active map reset |
| `Tracking.FramesToResetIMU` | 100000 | effectively disables `mnFramesToResetIMU` |
| `Tracking.MinInliersImuMonoPostInit` | 0 | keep tracking alive through brief visual dropouts post-init |
| `IMU.NoiseGyro / NoiseAcc / GyroWalk / AccWalk` | datasheet × 10 | conservative for non-lab use |

In the wrapper (C++ defaults set in `imu_mono_realsense.cpp`):

| Param | Value | Meaning |
|---|---|---|
| Image queue | size-1 latest-only | Matches `zang09/ORB_SLAM3_ROS2` and upstream `Examples/ROS/ORB_SLAM3` `ros_mono_inertial.cc` |
| `publish_pose_only_when_tracking_ok` | true | Don't publish a pose if tracking is RECENTLY_LOST |
| `min_pose_publish_inliers` | 0 | Don't gate on `mnMatchesInliers` (unreliable on IMU-mono fast paths) |
| `min_pose_publish_tracked_mappoints` | 0 | Same |

## If init keeps failing on the live system

Diagnose in this order:

1. Check `/sync/imu` `topic hz` ≈ 229 (or close). If it is jittering wildly or
   dropping, the issue is `cam_imu_sync`, not ORB.
2. Check `/sync/image_raw` `topic hz` ≈ 30. If it is < 10, the wrapper will
   starve.
3. Move the camera **more slowly** for the first 30 s. The window we set
   (8 s, 15 KFs) caps the worst case at ~12 s of motion — well within ORB's
   ability to estimate bias **if** gyro stays under 18 deg/s.
4. Check `IMU.NoiseGyro` / `NoiseAcc` are reasonable for your IMU. The
   datasheet × 10 setting we use is robust; if you tightened them to chase
   accuracy, init will be more fragile.
5. Read the relevant `[orb_event]` lines:
   - `MONO_INIT_FAILED_NOT_ENOUGH_MOTION` with `median_parallax < 1.0` → walk
     more (translation, not rotation).
   - `MONO_INIT_FAILED_NOT_ENOUGH_MOTION` with `reason=not_enough_triangulated`
     → scene is too uniform; point the camera somewhere with more texture.
   - `IMU_INIT_SUCCESS` then a post-init `Reseting active map...` →
     bias still wrong; restart and walk slower.

## What to do when you record a new bag for SLAM testing

Reuse the same motion script as live. We have a validated reference recording
at `bags/goalDebug/h30_150hz_slowMove` (99 s, 30 fps, 229 Hz IMU, gentle
motion throughout). It is the regression case for any future wrapper change.

End-to-end validation expectations (this is the slowMove run that passed):

| Metric | Expected |
|---|---|
| `IMU_INIT_SUCCESS` events | 1 (no re-init) |
| Total ORB-SLAM3 "Reseting active map..." prints | ≤ 5 (all pre-init) |
| Post-init resets | **0** |
| Max active maps | 1 (`map_id` never grows past 0) |
| Final pose drift | < 1 m (slowMove had 0.065 m) |
| Final keyframes | 50+ (slowMove had 62) |
| `/orb_odom` rate | stable 10 Hz |
