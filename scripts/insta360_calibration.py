"""
Build a SLAM calibration dir for one Insta360 camera from one of its .insv clips.

Writes to <output_dir>:
  slam_setting.yaml        ORB_SLAM3 KannalaBrandt8 setting, passed to 02/03 via --setting
  camera_intrinsics.json   same lens in the OpenICC json format 04_detect_aruco.py reads
  slam_mask.png            masks the image outside --max_fov_deg
  aruco_config.yaml        copied from example/calibration
  cam_imu_report.json      how IMU.T_b_c1 was obtained

Intrinsics come from Insta360's factory calibration stored in the file.
The camera-IMU rotation is estimated by matching gyro against visual rotation, so
give it a clip where the camera is rotated briskly about all three axes for 20-60 s.

python scripts/insta360_calibration.py -i VID_xxx_00_001.insv -o data/insta360_calib --lens 1
"""
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import json
import pathlib
import shutil
import click
import cv2
import numpy as np
from umi.common.insta360_util import (
    find_recordings, read_insv, fit_kannala_brandt, lens_mask, orbslam_settings_yaml, openicc_intrinsics_json, DEG_TO_RAD)

# Fallback when the clip has too little rotation to estimate from. Lens 1 is the
# Basalt camera-IMU rotation of X3 IAQEB2606EEWG9 (RoboManipBaselines
# envs/real/insta360_bridge/Insta360_X4.yaml, back lens), mapped from the live SDK
# IMU axes to telemetry-parser's: normalized = [[0,-1,0],[-1,0,0],[0,0,-1]] @ sdk,
# found by matching gravity at the same rest pose. It agrees within 1.5 deg with the
# gravity-only guess. Lens 0 is the same rotated 180 deg about the camera y axis.
# The translation (meters, camera origin in the IMU frame) is Basalt's, mapped the same
# way. Basalt flagged its run as under-excited; lens 0 has no measurement and uses 0.
_R_LENS1 = np.array([
    [-0.0018413, 0.0109166, -0.9999390],
    [0.0243852, -0.9996430, -0.0109582],
    [-0.9997010, -0.0244039, 0.0015744]])
DEFAULT_R_IMU_CAM = {1: _R_LENS1, 0: _R_LENS1 @ np.diag([-1.0, 1.0, -1.0])}
DEFAULT_T_IMU_CAM = {1: np.array([-0.127618, 0.165386, 0.022312]), 0: np.zeros(3)}
# Red gripper mount and fingers in lens 1 on our X3 mount, as fractions of image width and
# height, traced from a mapping frame with margin for the fingers opening. Rough; refine as needed.
GRIPPER_POLYGON = [(0.16, 1.0), (0.16, 0.74), (0.30, 0.60), (0.70, 0.60), (0.84, 0.74), (0.84, 1.0)]
MIN_ROTATION_RAD_S = 0.5
MIN_PAIRS = 30


def track_rotations(clip, kb, step, max_theta):
    """Per frame pair (t0_s, t1_s, rotation vector of the camera in its own frame)."""
    lens_size = clip.lens.size
    scale = 960 / lens_size
    kb_s = kb.scaled(960, 960)
    K = np.array([[kb_s.fx, 0, kb_s.cx], [0, kb_s.fy, kb_s.cy], [0, 0, 1]])
    D = np.array(kb_s.k)
    r_max = kb_s.radius_px(max_theta)
    cap = cv2.VideoCapture(str(clip.video_path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    prev, prev_idx, out, idx = None, None, [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % step == 0:
            x0 = clip.crop_x
            gray = cv2.cvtColor(frame[:, x0:x0 + lens_size], cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            if prev is not None:
                rvec = _pair_rotation(prev, gray, K, D, r_max)
                if rvec is not None:
                    out.append((prev_idx / fps, idx / fps, rvec))
            prev, prev_idx = gray, idx
        idx += 1
    return out


def _pair_rotation(img0, img1, K, D, r_max):
    pts0 = cv2.goodFeaturesToTrack(img0, 600, 0.01, 12)
    if pts0 is None:
        return None
    pts1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img1, pts0, None, winSize=(21, 21), maxLevel=4)
    good = st.ravel() == 1
    c = K[:2, 2]
    inside = (np.linalg.norm(pts0[:, 0] - c, axis=1) < r_max) & (np.linalg.norm(pts1[:, 0] - c, axis=1) < r_max)
    good &= inside
    if good.sum() < 40:
        return None
    b0 = _bearings(cv2.fisheye.undistortPoints(pts0[good], K, D))
    b1 = _bearings(cv2.fisheye.undistortPoints(pts1[good], K, D))
    # Between nearby frames of a handheld clip, translation is small next to scene depth,
    # so fit a pure rotation b1 = R b0. The essential matrix is degenerate in that case.
    inl = np.ones(len(b0), dtype=bool)
    for _ in range(3):
        R = solve_rotation(b1[inl], b0[inl])
        err = np.arccos(np.clip(np.sum(b1 * (b0 @ R.T), axis=1), -1, 1))
        inl = err < max(np.median(err) * 3, 2e-3)
    if inl.sum() < 30:
        return None
    # b1 = R b0 for static points, so the camera itself rotated by R^T.
    rvec, _ = cv2.Rodrigues(R.T)
    return rvec.ravel()


def _bearings(normalized):
    v = np.concatenate([normalized.reshape(-1, 2), np.ones((normalized.shape[0], 1))], axis=1)
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def integrate_gyro(t_s, gyro, t0, t1):
    sel = (t_s >= t0) & (t_s < t1)
    if sel.sum() < 2:
        return None
    return gyro[sel].mean(axis=0) * (t1 - t0)


def solve_rotation(cam_rv, imu_rv):
    """R minimizing sum |cam - R imu|^2 with det(R) = +1."""
    U, _, Vt = np.linalg.svd(cam_rv.T @ imu_rv)
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ S @ Vt


def estimate_cam_imu(clip, kb, step, max_theta):
    pairs = track_rotations(clip, kb, step, max_theta)
    t_s = clip.imu_t_ms / 1000.0
    best = None
    for offset_ms in range(-60, 61, 2):
        dt = offset_ms / 1000.0
        cam, imu = [], []
        for t0, t1, rv in pairs:
            g = integrate_gyro(t_s, clip.gyro, t0 + dt, t1 + dt)
            if g is not None and np.linalg.norm(g) / (t1 - t0) > MIN_ROTATION_RAD_S:
                cam.append(rv)
                imu.append(g)
        if len(cam) < MIN_PAIRS:
            continue
        cam, imu = np.array(cam), np.array(imu)
        R_cam_imu = solve_rotation(cam, imu)
        resid = np.linalg.norm(cam - imu @ R_cam_imu.T, axis=1)
        score = float(np.median(resid / np.linalg.norm(cam, axis=1)))
        if best is None or score < best['median_relative_residual']:
            best = {'R_cam_imu': R_cam_imu, 'median_relative_residual': score,
                    'imu_time_offset_ms': offset_ms, 'n_pairs': len(cam), 'n_tracked_pairs': len(pairs)}
    return best


@click.command()
@click.option('-i', '--input', 'insv_path', required=True, help='Any file of a take from the camera to calibrate (.insv or its LRV)')
@click.option('-o', '--output_dir', required=True)
@click.option('-l', '--lens', type=int, default=1, help='Lens index the pipeline uses (00_convert_insv.py --lens)')
@click.option('--slam_size', type=int, default=960, help='Square image size ORB_SLAM3 tracks at')
@click.option('--max_fov_deg', type=float, default=95.0, help='Mask out rays beyond this angle off the optical axis')
@click.option('--no_gripper_mask', is_flag=True, default=False, help='Do not mask GRIPPER_POLYGON')
@click.option('--frame_step', type=int, default=2, help='Estimate visual rotation every N frames')
def main(insv_path, output_dir, lens, slam_size, max_fov_deg, no_gripper_mask, frame_step):
    insv_path = pathlib.Path(insv_path).absolute()
    rec = next((r for r in find_recordings(insv_path.parent, lens) if insv_path in r.files), None)
    if rec is None:
        raise SystemExit(f'{insv_path} is not part of a take found in {insv_path.parent}')
    clip = read_insv(rec.telemetry, lens, rec.video)
    out = pathlib.Path(os.path.expanduser(output_dir)).absolute()
    out.mkdir(parents=True, exist_ok=True)

    kb = fit_kannala_brandt(clip.lens)
    print(f'{clip.camera_model} {clip.serial} lens {lens}: Kannala-Brandt fit max error {kb.max_fit_error_px:.3f} px '
          f'at {kb.width}x{kb.height}')
    max_theta = max_fov_deg * DEG_TO_RAD

    est = estimate_cam_imu(clip, kb, frame_step, max_theta)
    report = {'insv': str(clip.video_path), 'serial': clip.serial, 'lens': lens}
    if est is None:
        R_imu_cam = DEFAULT_R_IMU_CAM[lens]
        report['source'] = 'default: clip had too little rotation to estimate from; ' + ('Basalt calibration of X3 IAQEB2606EEWG9' if lens == 1 else 'unverified guess')
        print(f'WARNING: fewer than {MIN_PAIRS} frame pairs rotating faster than {MIN_ROTATION_RAD_S} rad/s. '
              f"Using the default camera-IMU rotation ({report['source'].split('; ')[1]}).")
    else:
        R_imu_cam = est['R_cam_imu'].T
        report.update(source='estimated from gyro vs visual rotation',
                      median_relative_residual=est['median_relative_residual'],
                      imu_time_offset_ms=est['imu_time_offset_ms'],
                      n_pairs=est['n_pairs'], n_tracked_pairs=est['n_tracked_pairs'])
        print(f"Estimated R_imu_cam from {est['n_pairs']} pairs, median relative residual "
              f"{est['median_relative_residual']:.3f}, best IMU time offset {est['imu_time_offset_ms']} ms")
    print('R_imu_cam =\n' + np.array2string(R_imu_cam, precision=4, suppress_small=True))
    T_b_c = np.eye(4)
    T_b_c[:3, :3] = R_imu_cam
    T_b_c[:3, 3] = DEFAULT_T_IMU_CAM[lens]
    report['T_b_c1'] = T_b_c.tolist()

    imu_hz = 1000.0 / float(np.median(np.diff(clip.imu_t_ms)))
    kb_slam = kb.scaled(slam_size, slam_size)
    (out / 'slam_setting.yaml').write_text(orbslam_settings_yaml(kb_slam, clip.fps, T_b_c, imu_hz))
    (out / 'camera_intrinsics.json').write_text(json.dumps(openicc_intrinsics_json(kb, clip.fps), indent=4))
    mask = lens_mask(kb, max_theta)
    if not no_gripper_mask:
        poly = np.round(np.array(GRIPPER_POLYGON) * [kb.width, kb.height]).astype(np.int32)
        cv2.fillPoly(mask, [poly], 255)
    cv2.imwrite(str(out / 'slam_mask.png'), mask)
    if not (out / 'aruco_config.yaml').exists():
        # Copied once; edit marker_size_map there to match your printed tags.
        shutil.copyfile(pathlib.Path(ROOT_DIR, 'example', 'calibration', 'aruco_config.yaml'), out / 'aruco_config.yaml')
    (out / 'cam_imu_report.json').write_text(json.dumps(report, indent=2))
    print(f'Wrote calibration to {out}')


if __name__ == '__main__':
    main()
