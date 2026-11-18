import pathlib
import subprocess

import pytest

from umi.common.insta360_util import _lens_model, find_recordings

# Factory calibration and sensor crop from X3 IAQEB2606EEWG9 file metadata.
X3_METADATA = {
    'offset_v3': [2.0, 1.94817, 4615.69, 4614.29, 2978.9, 3008.01, 0.216, 0.273, 90.36, 0.0, 0.0, 0.0,
                  0.37859267, 1.37801683, -4.17721272, -0.00065036, -0.00153947, 11904.0, 5952.0, 71.0,
                  1.94817, 4620.34, 4619.34, 8919.07, 2984.55, -0.294, 0.112, 88.246, -0.001054, -4.2e-05,
                  -0.027488, 0.40238833, 1.20215559, -3.80487084, 0.00107035, -0.00236576, 11904.0, 5952.0, 71.0,
                  199424.0],
    'window_crop_info': {'src_width': 5952, 'src_height': 5952, 'dst_width': 5760, 'dst_height': 5760,
                         'crop_offset_x': 0, 'crop_offset_y': 0},
}


def test_packed_lens_matches_gyroflow_camera_matrix():
    # gyroflow's matrix for the same 3840x1920 file: fy 1538.10, cy 970.67, and fx, cx doubled for the packed width.
    lens = _lens_model(X3_METADATA, 0, 1920)
    assert lens.fy == pytest.approx(1538.10, abs=0.01)
    assert lens.cy == pytest.approx(970.67, abs=0.01)
    assert 2 * lens.fx == pytest.approx(3077.13, abs=0.01)
    assert 2 * lens.cx == pytest.approx(1921.93, abs=0.01)


def test_packed_and_split_modes_share_normalized_intrinsics():
    packed, split = _lens_model(X3_METADATA, 1, 1920), _lens_model(X3_METADATA, 1, 2880)
    for attr in ('fx', 'fy', 'cx', 'cy'):
        assert getattr(packed, attr) / 1920 == pytest.approx(getattr(split, attr) / 2880)


def make_video(path: pathlib.Path, width: int, height: int):
    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i', f'testsrc=d=0.2:s={width}x{height}:r=10',
                    '-c:v', 'libx264', '-f', 'mp4', str(path)], check=True)


def test_find_recordings_groups_packed_split_and_mapping_takes(tmp_path):
    raw = tmp_path / 'raw_videos'
    make_video(raw / 'VID_20261119_020417_00_007.insv', 64, 32)
    make_video(raw / 'LRV_20261119_020417_01_007.lrv', 32, 16)
    make_video(raw / 'mapping' / 'VID_20261119_032839_00_013.insv', 32, 32)
    make_video(raw / 'mapping' / 'VID_20261119_032839_10_013.insv', 32, 32)
    make_video(raw / 'mapping' / 'LRV_20261119_032839_11_013.lrv', 32, 16)
    make_video(tmp_path / 'raw_insv' / 'VID_20261119_010000_00_001.insv', 64, 32)

    recs = {r.out_mp4.relative_to(tmp_path).as_posix(): r
            for r in find_recordings(tmp_path, lens_idx=1, exclude_dir=tmp_path / 'raw_insv')}

    assert set(recs) == {'raw_videos/VID_20261119_020417_00_007.mp4', 'raw_videos/mapping.mp4'}
    packed = recs['raw_videos/VID_20261119_020417_00_007.mp4']
    assert packed.video.name == packed.telemetry.name == 'VID_20261119_020417_00_007.insv'
    split = recs['raw_videos/mapping.mp4']
    assert split.video.name == 'VID_20261119_032839_10_013.insv'
    assert split.telemetry.name == 'LRV_20261119_032839_11_013.lrv'
    assert len(split.files) == 3


def test_split_take_without_lrv_is_rejected(tmp_path):
    make_video(tmp_path / 'VID_20261119_032839_10_013.insv', 32, 32)
    with pytest.raises(ValueError, match='LRV'):
        find_recordings(tmp_path, lens_idx=1)

