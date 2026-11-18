"""
Convert Insta360 .insv clips into the GoPro-shaped inputs of the SLAM pipeline.
For every <name>.insv under the session dir, writes next to it:
  <name>.mp4            one lens, tagged with camera serial and start timecode
  <name>.imu_data.json  IMU in the schema gopro_slam reads
00_process_videos.py then moves both into demos/<demo>/.
Converted .insv files move to <session_dir>/raw_insv/, which is never scanned, so reruns skip them.

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
from umi.common.insta360_util import read_insv, imu_to_gpmf_json, write_lens_mp4

IMU_SIDECAR_SUFFIX = '.imu_data.json'
DONE_DIR = 'raw_insv'


@click.command()
@click.argument('session_dir', nargs=-1)
@click.option('-l', '--lens', type=int, default=1, help='Lens index to keep. 1 faces the gripper on our mount.')
def main(session_dir, lens):
    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute()
        done_dir = session.joinpath(DONE_DIR)
        insv_paths = sorted(p for p in session.glob('**/*') if p.suffix.lower() == '.insv'
                            and done_dir not in p.parents)
        print(f'Found {len(insv_paths)} unconverted insv files in {session}')
        for insv_path in insv_paths:
            clip = read_insv(insv_path)
            insv_path.with_name(insv_path.stem + IMU_SIDECAR_SUFFIX).write_text(json.dumps(imu_to_gpmf_json(clip)))
            write_lens_mp4(clip, lens, insv_path.with_suffix('.mp4'))
            done_dir.mkdir(exist_ok=True)
            shutil.move(insv_path, done_dir.joinpath(insv_path.name))
            print(f'{insv_path.name}: {clip.n_frames} frames, {len(clip.imu_t_ms)} IMU samples, serial {clip.serial}')


if __name__ == '__main__':
    if len(sys.argv) == 1:
        main.main(['--help'])
    else:
        main()
