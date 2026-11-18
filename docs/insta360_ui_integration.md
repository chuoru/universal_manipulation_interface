# Insta360 capture UI: integration guide

For the agent building the UI on top of the UMI Insta360 pipeline. It says which camera functions and which pipeline commands each screen wraps, how to run them, what output to expect, and what does not exist yet.

Everything marked **verified** ran on a real Insta360 X3 (serial `IAQEB2606EEWG9`, firmware v1.1.6) on 2026-11-19. Everything marked **unverified** or **new** has not.

## 1. What the UI is for

A person holding a UMI gripper with an Insta360 X3 on it records takes, gets a verdict on each take within seconds, and turns a session into a training dataset without a terminal.

There are two ways to record, and they converge as soon as the file reaches the PC:

| Mode | How a take reaches the PC | Mode-specific screens |
|---|---|---|
| Wired | Camera on USB; the PC starts and stops recording, then downloads the take | Live capture |
| Offline | Camera records with its own button, untethered; later the PC lists the card and imports selected takes | Arm camera, Import from card |

Shared by both: session home, take review, processing, and the components on the design canvas.

## 2. Design

Canvas (Claude artifact, private to the owner's account; any of the owner's Claude sessions can read it with the Artifact tool):

```
https://claude.ai/artifact/5QugK1H31uHWBhW8w8nccx
```

Read `project/canvas.json` first, then the artboards it lists. Screens:

| Artboard | Screen |
|---|---|
| `Main.dc.html` | Session home: phases, "Record now", "Import from camera card", demo list |
| `Processing.dc.html` | Takes × pipeline stages matrix, map quality, dataset |
| `WiredCapture.dc.html` | Live view, role picker, role guide, live checks, Stop |
| `WiredReview.dc.html` | Take review, shared by both modes; a `mode` prop switches wired / offline |
| `OfflineArm.dc.html` | Prepare the camera before unplugging it |
| `OfflineImport.dc.html` | Takes on the card with preview checks and role pickers |
| `SharedComponents.dc.html` | Gallery of the components below |

Components (linked artboards, mounted with `<dc-import>`): `StatusChip`, `CameraBar`, `TakeRow`, `CheckList`, `RolePicker`, `RoleGuide`, `TransferQueue`. Their props are the data contract the UI has to supply; see each file's `data-props` and default `renderVals()`.

## 3. Environments

Two Python environments are required, because pyinsta360 needs Python 3.10+ and the pipeline pins Python 3.9:

| Environment | Python | Used for | Interpreter |
|---|---|---|---|
| pyinsta360 venv | 3.14 | Every camera call | `~/Work/rikyu/pyinsta360/.venv/bin/python` |
| `umi` conda env | 3.9 | Conversion, checks, SLAM pipeline, dataset | `~/miniconda3/envs/umi/bin/python` |

The UI backend must run camera work and pipeline work as separate processes. Do not import `umi/common/insta360_util.py` (needs telemetry-parser) from the pyinsta360 venv, or `umi/common/insta360_camera.py` (needs pyinsta360) from the `umi` env. `umi/common/insta360_files.py` has no dependencies and is safe in both.

Other requirements: Docker with image `chicheng/orb_slam3:latest` (pulled), `ffmpeg`, and the camera in **Android USB mode** (see 6.1).

Repo: `~/Work/rikyu/universal_manipulation_interface`, branch `insta360` on `https://github.com/chuoru/universal_manipulation_interface`.

## 4. Camera functions

Call these through `umi/common/insta360_camera.py`, not the SDK directly: the helpers hold the workarounds for the SDK bugs in section 6.

| Screen / component | Function | Status |
|---|---|---|
| CameraBar: connected, battery, card | `pi.discover()`, `pi.Camera()` inside `pi.run_supervised(...)`, `cam.info`, `cam.battery_status()`, `cam.storage_state()` | Verified |
| CameraBar: "camera is a USB drive" | Not available from the SDK (it only returns 0 cameras). Check USB at the OS level: `2e1a:0002 Arashi Vision Insta360 X3` = Android mode; a mass-storage device `070a:4026 AmbarellaInc A9 Platform` = U-disk mode | **New** (checked by hand with `lsusb`) |
| Arm: clock, mode | `cam.sync_local_time(utc_seconds, offset_seconds)`, `cam.current_function_mode()` | Verified. The mode can be read, not set: it reported `FUNCTION_MODE_INTERVAL_SHOOTING` even after pushing video params |
| Live capture: record | `cam.set_video_capture_params(pi.RecordParams(pi.VideoResolution.RES_3840_1920P30), pi.CameraFunctionMode.FUNCTION_MODE_NORMAL_VIDEO)`, `cam.start_recording()`, `cam.stop_recording()`, then diff `cam.files_list()` before/after | Verified. Wrapped by `record()` in `scripts/insta360_record.py` |
| Live capture: live view, live tag checks | `cam.start_live_streaming()`, `cam.read_video()` + `pyinsta360.decode.VideoDecoder`, `cam.read_imu()`, `cam.stream_stats()` | **Unverified.** Unknown whether the camera streams while recording |
| Import: list card | `list_camera_files()`, `group_takes(files)` → `CameraTake(index, started, vids, lrv)` | Verified |
| TransferQueue | `fetch_take(take, role, session, lens_idx)` → `download_with_retries(remote, local)` | Verified, including stall recovery |
| Not used | `delete_file()` (only for a future explicit "Free up card"), `take_photo()`, exposure settings | n/a |

## 5. Pipeline commands

All run in the `umi` env from the repo root unless marked otherwise.

| Screen | Command / function | Output the UI reads | Status |
|---|---|---|---|
| Rig setup | `python scripts/insta360_calibration.py -i <any file of a take> -o <calib_dir> --lens 1` | `cam_imu_report.json` (`source` says estimated vs default), `slam_setting.yaml`, `camera_intrinsics.json`, `slam_mask.png`, `aruco_config.yaml` | Verified |
| Rig setup: tag sizes | Edit `<calib_dir>/aruco_config.yaml` `marker_size_map` (finger tags 0, 1; table tag 13). Reruns keep this file | Sizes used by steps 04/05 | Verified (our print: 0.015 m and 0.155 m) |
| Rig setup: mask | `GRIPPER_POLYGON` in `scripts/insta360_calibration.py`, or paint `slam_mask.png` (255 = ignored) | Mask used by steps 02/03 | Verified; the polygon is rough |
| Live capture | (pyinsta360 venv) `python scripts/insta360_record.py <session> --kind {mapping,gripper_calibration,demo} [--seconds N]` | Take files in the role folder (7.1) | Verified |
| Import | (pyinsta360 venv) `python scripts/insta360_import.py <session> --list` and `... --mapping 13 --gripper-calibration 11 --demo 12 14-20` | Take files in the role folders | Verified |
| Take review: checks | Finger-tag visibility (IDs 0, 1, threshold 80% of frames), table tag (ID 13) at start and end, IMU coverage, length, read from the take's LRV proxy | Pass / check / fail per rule | **New**: done by hand in this session, not in the repo |
| Processing: convert | `python scripts_slam_pipeline/00_convert_insv.py <session> --lens 1` | `.mp4` + `.imu_data.json` per take; camera files moved to `raw_insv/` | Verified |
| Processing: per stage | `00_process_videos.py <session>`, `01_extract_gopro_imu.py <session>` (skips itself), `02_create_map.py -i <session>/demos/mapping --map_path <...>/map_atlas.osa --setting <calib>/slam_setting.yaml --mask <calib>/slam_mask.png`, `03_batch_slam.py -i <session>/demos --map_path <...> --setting ... --mask ...`, `04_detect_aruco.py -i <session>/demos -ci <calib>/camera_intrinsics.json -ac <calib>/aruco_config.yaml`, `05_run_calibrations.py <session>`, `06_generate_dataset_plan.py --input <session>` | Per take: `camera_trajectory.csv` (column `is_lost`), `tag_detection.pkl`, `gripper_range.json`; session: `demos/mapping/tx_slam_tag.json`, `dataset_plan.pkl` | Verified |
| Processing: all at once | `python run_slam_pipeline.py <session> -c <calib_dir>` | Same files | Verified (exit 0, "99% of raw data are used", `n_dropped_demos 0`) |
| Processing: map quality | Scale of SLAM vs the table tag's known size, and tag world-position spread | e.g. "steady to 1.3 cm", "4.2% short" | **New**: one-off script, not in the repo |
| Dataset | `python scripts_slam_pipeline/07_generate_replay_buffer.py -o <session>/dataset.zarr.zip <session>` | `dataset.zarr.zip`: `camera0_rgb (N,224,224,3)`, `robot0_eef_pos`, `robot0_eef_rot_axis_angle`, `robot0_gripper_width`, episode ends | Verified |

Every step skips work already done, so the UI can rerun a single failed stage. The scripts report progress only as printed text.

## 6. Behaviour the UI must handle

### 6.1 Camera connection
- The camera enumerates as a USB drive by default. The user selects **Android** under Settings › General › USB Mode, then replugs.
- It leaves Android mode when it sleeps. `lsusb` still shows it but the SDK finds nothing. Fix: wake it and reselect Android.
- `discover()` / `Camera()` sometimes aborts the process with `std::bad_function_call` (SDK 2.1.8 bug). `run_supervised` retries; the download watchdog retries too.

### 6.2 Recording and downloading
- `stop_recording()` returns the wrong file path. Diff `files_list()` instead (done by `record()`).
- The first `download_file` after a take often sits at 0 bytes forever. The watchdog kills an attempt whose `.part` file has not grown for 20 s and retries (up to 4 attempts). Healthy downloads write about 18 MB/s.
- Downloads land as `<name>.part` and are renamed when complete. Nothing is ever deleted from the camera.

### 6.3 Recording modes and file layouts
| Layout | When | Files per take | Converter |
|---|---|---|---|
| Packed | Takes started over USB (SDK applies `RES_3840_1920P30`) | `VID_..._00_NNN.insv` (3840×1920, both lenses) + `LRV_..._01_NNN.lrv` | Verified |
| Split | Button presses in 5.7K 30 fps | `VID_..._00_NNN.insv` + `VID_..._10_NNN.insv` (2880×2880 each, one lens) + `LRV_..._11_NNN.lrv`; IMU and calibration are only in the LRV | Verified (lens file + LRV) |
| 60 fps | Button presses in any 60 fps mode | 2880×1440 per lens | Rejected by the converter |
| 4K 30 fps from the button | Unknown whether it is packed | | **Unverified** |

Lens 1 (`_10_` file in split takes) faces the gripper on this mount and is the default.

### 6.4 Time
- Start time comes from the camera file name (local camera clock), which matched the host to within 1 s. `creation_time` inside the file runs about 10 s early and is UTC for SDK-started takes but local time for button-started ones; do not use it.
- `umi/common/timecode_util.py` shifts read-back start times by about 66 s for 29.97 fps video (a pre-existing UMI quirk, consistent within a session).

### 6.5 Session layout and roles
- One mapping take per session, in `raw_videos/mapping/`. Both camera tools refuse a second one.
- Gripper calibration in `raw_videos/gripper_calibration/`. If it is missing, step 00 silently files the earliest demo as the calibration take.
- A demo counts only if finger tags 0 and 1 are both visible in at least 80% of its frames (step 06). Otherwise step 06 fails with "No gripper camera detected!".
- Finger tags must sit at 7.2 ± 0.8 cm depth from the camera or they are ignored (`nominal_z`). Ours measure 6.75 cm, inside the window.

## 7. Test data on this machine

| Path | Contents |
|---|---|
| `data_workspace/insta360_20261119/` | Processed session: mapping (take 8), gripper calibration (11), demos 12 (wired) and 13 (offline 5.7K), `dataset_plan.pkl`, `dataset.zarr.zip` |
| `data_workspace/insta360_calib/` | Calibration dir used for that session |
| `data_workspace/insta360_20261119_untagged/` | Demo 9, recorded before finger tags were attached (expected to fail checks) |
| Camera card | Takes 2–14; see section 8 for expected check results |

Expected values for regression checks:

| Item | Expected |
|---|---|
| Mapping SLAM | 1811 / 1811 frames tracked |
| Demo 12 SLAM | 906 / 906 tracked; finger tags 100% |
| Demo 13 SLAM | 1042 / 1042 tracked; tag 0 100%, tag 1 90%, table tag 99% |
| Gripper range | `min_width` 0.0510, `max_width` 0.0848 m (width 0 to 3.4 cm) |
| Table tag | position spread about 1.3 cm; SLAM scale about 4.2% short of the 155 mm tag |
| Dataset | 2 episodes, 1943 steps |

## 8. Take review cases from the card

Lengths and tag percentages were measured. The verdicts are predictions under the rules in 6.5: the start-and-end table-tag rule has not been run on any take, only overall table-tag visibility.

| Take | Started | Length | Predicted verdict |
|---|---|---|---|
| 7 | 02:04:17 | 6 s | Check: too short |
| 9 | 02:13:33 | 30 s | Fail: finger tags 0% |
| 10 | 02:49:45 | 5 s | Fail: tag 1 53%, no table tag; Check: too short |
| 12 | 03:04:55 | 30 s | Pass |
| 13 | 03:28:39 | 35 s (5.7K split) | Pass |
| 14 | 03:51:56 | 3 s | Check: too short |

## 9. Gaps to build or settle

1. **Take-check function** (umi env): rules from section 6.5 over a take's LRV proxy, returning a list matching `CheckList`'s `checks` prop. Feeds take review and the import list in both modes.
2. **Map-quality check** (umi env): SLAM scale and table-tag spread from `mapping_camera_trajectory.csv` + tag detections.
3. **USB-mode detection** (OS level): drives CameraBar's attention states.
4. **Live preview while recording**: test before relying on the live view and live tag chips. If it does not work, the capture screen falls back to a pre-roll preview plus the post-take review.
5. **Structured progress** from pipeline steps (today only printed text).
6. **Role cards** (ArUco IDs 20+ held to the camera at take start): an idea; no detection code exists.
