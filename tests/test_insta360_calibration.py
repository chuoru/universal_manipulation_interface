import importlib.util
import pathlib

import cv2
import numpy as np
import pytest

from umi.common.insta360_util import MeiLens, fit_kannala_brandt

ROOT = pathlib.Path(__file__).parent.parent
spec = importlib.util.spec_from_file_location('insta360_calibration', ROOT / 'scripts' / 'insta360_calibration.py')
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)

# Lens 1 of an Insta360 X3, factory calibration scaled to a 1920 px lens image.
X3_LENS = MeiLens(xi=1.94817, fx=1490.4, fy=1490.1, cx=957.1, cy=962.8,
                  k=(0.40238833, 1.20215559, -3.80487084), p=(0.00107035, -0.00236576), size=1920)


@pytest.fixture(scope='module')
def camera():
    kb = fit_kannala_brandt(X3_LENS).scaled(960, 960)
    K = np.array([[kb.fx, 0, kb.cx], [0, kb.fy, kb.cy], [0, 0, 1]])
    D = np.array(kb.k)
    texture = cv2.GaussianBlur(np.random.default_rng(0).integers(0, 255, (960, 960)).astype(np.uint8), (0, 0), 3)
    texture = cv2.normalize(texture, None, 0, 255, cv2.NORM_MINMAX)
    return kb, K, D, texture


def render_rotated(img, K, D, rvec_cam):
    """The view after the camera rotates by rvec_cam (in its own frame)."""
    R_cam = cv2.Rodrigues(np.asarray(rvec_cam, float))[0]
    yy, xx = np.mgrid[:960, :960].astype(np.float32)
    bearings = cal._bearings(cv2.fisheye.undistortPoints(np.stack([xx.ravel(), yy.ravel()], 1)[:, None], K, D))
    old = bearings @ R_cam.T
    src = cv2.fisheye.distortPoints((old[:, :2] / old[:, 2:3])[:, None], K, D).reshape(-1, 2).astype(np.float32)
    src[bearings[:, 2] < 0.3] = -1
    return cv2.remap(img, src[:, 0].reshape(960, 960), src[:, 1].reshape(960, 960), cv2.INTER_LINEAR)


def test_kannala_brandt_matches_factory_model():
    kb = fit_kannala_brandt(X3_LENS)
    assert kb.max_fit_error_px < 0.5
    theta = np.deg2rad(100)
    assert kb.radius_px(theta) == pytest.approx(X3_LENS.project_radius(theta) * X3_LENS.fx, abs=0.5)


@pytest.mark.parametrize('rvec_deg', [(0, 0, 3), (0, 4, 0), (3, 0, 0), (2, -2, 1), (0, 0, -0.5)])
def test_visual_rotation_recovers_camera_motion(camera, rvec_deg):
    kb, K, D, img = camera
    rvec = np.deg2rad(rvec_deg)
    est = cal._pair_rotation(img, render_rotated(img, K, D, rvec), K, D, kb.radius_px(np.deg2rad(80)))
    assert np.rad2deg(np.abs(est - rvec)).max() < 0.1


def test_procrustes_recovers_cam_imu_rotation():
    R = cv2.Rodrigues(np.array([0.3, -1.1, 0.7]))[0]
    imu = np.random.default_rng(1).normal(size=(50, 3))
    assert np.abs(cal.solve_rotation(imu @ R.T, imu) - R).max() < 1e-9
