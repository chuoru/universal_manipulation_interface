"""
Record a take on a USB-connected Insta360 and download it into a UMI session.

Run with the Python that has pyinsta360 installed (it needs Python 3.10+, the umi env is 3.9).
The camera must be in Android USB mode, not U-disk mode.

  <session>/raw_videos/mapping/<VID>.insv              --kind mapping
  <session>/raw_videos/gripper_calibration/<VID>.insv  --kind gripper_calibration
  <session>/raw_videos/<VID>.insv                      --kind demo (default)

run_slam_pipeline.py then converts each take to .mp4 + imu_data.json (00_convert_insv.py).
Files stay on the camera. If a download fails, fetch the take later with
scripts/insta360_import.py <session> --demo <take index>.

python scripts/insta360_record.py data/session1 --kind mapping
python scripts/insta360_record.py data/session1 --seconds 20
"""
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse
import pathlib
import time

import pyinsta360 as pi

from umi.common.insta360_camera import ROLES, fetch_take, group_takes, mapping_exists

# Packed 3840x1920 at 30 fps. The SDK applies this to takes it starts, though not to button presses.
RESOLUTION = pi.VideoResolution.RES_3840_1920P30


def wait_for_stop(seconds):
    if seconds is not None:
        time.sleep(seconds)
        return
    # run_supervised's child has no stdin, so read the key press from the terminal directly.
    with open('/dev/tty') as tty:
        print('Recording. Press Enter to stop.', flush=True)
        tty.readline()


def record(seconds):
    """Record one take and return the camera files it created."""
    pi.set_log_level(pi.LogLevel.WARNING)
    with pi.Camera() as cam:
        print(f'Camera {cam.info}, battery {cam.battery_status()}', flush=True)
        offset = -time.altzone if time.localtime().tm_isdst else -time.timezone
        cam.sync_local_time(int(time.time()), offset)
        cam.set_video_capture_params(pi.RecordParams(RESOLUTION), pi.CameraFunctionMode.FUNCTION_MODE_NORMAL_VIDEO)

        before = set(cam.files_list())
        cam.start_recording()
        print(f'recording started at host time {time.strftime("%Y-%m-%d %H:%M:%S")}', flush=True)
        wait_for_stop(seconds)
        cam.stop_recording()
        # stop_recording() returns the wrong path on SDK 2.1.8 (pyinsta360 README), so diff the file list.
        return sorted(set(cam.files_list()) - before)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir')
    ap.add_argument('--kind', choices=ROLES, default='demo')
    ap.add_argument('--seconds', type=float, default=None, help='Stop after this long instead of on Enter')
    ap.add_argument('--lens', type=int, default=1, help='Lens the pipeline keeps; picks the file of a split take')
    args = ap.parse_args()

    session = pathlib.Path(args.session_dir).expanduser().absolute()
    if args.kind == 'mapping' and mapping_exists(session):
        raise SystemExit(f'{session} already has a mapping take; a session has one mapping video.')
    takes = group_takes(pi.run_supervised(record, args.seconds))
    if len(takes) != 1:
        raise SystemExit(f'expected one new take on the camera, found {len(takes)}: {[t.index for t in takes]}')
    fetch_take(takes[0], args.kind, session, args.lens)


if __name__ == '__main__':
    main()
