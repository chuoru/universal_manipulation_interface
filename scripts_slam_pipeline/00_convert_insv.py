"""
Convert Insta360 takes into the GoPro-shaped inputs of the SLAM pipeline.
Handles both layouts: packed (one .insv, both lenses side by side, e.g. 3840x1920) and
split (one .insv per lens, e.g. 5.7K, with IMU and calibration read from the LRV proxy).
For every take under the session dir, writes next to it:
  <VID_...>.mp4            one lens, tagged with camera serial and start timecode
  <VID_...>.imu_data.json  IMU in the schema gopro_slam reads
A take inside raw_videos/mapping/ becomes raw_videos/mapping.mp4.
00_process_videos.py then moves both into demos/<demo>/.
Converted camera files move to <session_dir>/raw_insv/<same relative path>, which is never
scanned, so reruns skip them.

python scripts_slam_pipeline/00_convert_insv.py data_workspace/cup/20261119 --lens 1
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
from umi.common.insta360_util import find_recordings, read_insv, imu_to_gpmf_json, write_lens_mp4

IMU_SIDECAR_SUFFIX = '.imu_data.json'
DONE_DIR = 'raw_insv'


@click.command()
@click.argument('session_dir', nargs=-1)
@click.option('-l', '--lens', type=int, default=1, help='Lens index to keep. 1 faces the gripper on our mount.')
def main(session_dir, lens):
    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute()
        done_dir = session.joinpath(DONE_DIR)
        recordings = find_recordings(session, lens, exclude_dir=done_dir)
        print(f'Found {len(recordings)} unconverted takes in {session}')
        for rec in recordings:
            clip = read_insv(rec.telemetry, lens, rec.video)
            sidecar = rec.out_mp4.with_name(rec.out_mp4.stem + IMU_SIDECAR_SUFFIX)
            sidecar.write_text(json.dumps(imu_to_gpmf_json(clip)))
            write_lens_mp4(clip, rec.out_mp4)
            for path in rec.files:
                target = done_dir.joinpath(path.relative_to(session))
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(path, target)
            print(f'{rec.out_mp4.relative_to(session)} <- {", ".join(p.name for p in rec.files)}: '
                  f'{clip.layout.value}, {clip.lens.size}px, {clip.n_frames} frames, '
                  f'{len(clip.imu_t_ms)} IMU samples, start {clip.start_datetime:%Y-%m-%d %H:%M:%S} UTC')


if __name__ == '__main__':
    if len(sys.argv) == 1:
        main.main(['--help'])
    else:
        main()
