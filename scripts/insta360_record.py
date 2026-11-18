"""
Record a clip on a USB-connected Insta360 and download its .insv into a UMI session.

Run with the Python that has pyinsta360 installed (it needs Python 3.10+, the umi env is 3.9).
The camera must be in Android USB mode, not U-disk mode.

  <session>/raw_videos/mapping.insv                    --kind mapping
  <session>/raw_videos/gripper_calibration/<VID>.insv  --kind gripper_calibration
  <session>/raw_videos/<VID>.insv                      --kind demo (default)

run_slam_pipeline.py then converts each .insv to .mp4 + imu_data.json (00_convert_insv.py).
Files stay on the camera. If a download fails, fetch it later with --download <camera path>.

python scripts/insta360_record.py data/session1 --kind mapping
python scripts/insta360_record.py data/session1 --seconds 20
"""
import argparse
import multiprocessing
import pathlib
import time

import pyinsta360 as pi

# The packed dual-fisheye layout 00_convert_insv.py reads, and the resolution
# the RoboManipBaselines camera-IMU calibration was made at.
RESOLUTION = pi.VideoResolution.RES_3840_1920P30
DOWNLOAD_SETTLE_S = 5
# The first download after a take often sits at 0 bytes forever; a healthy one writes ~18 MB/s.
DOWNLOAD_STALL_S = 20
DOWNLOAD_ATTEMPTS = 4


def wait_for_stop(seconds):
    if seconds is not None:
        time.sleep(seconds)
        return
    # run_supervised's child has no stdin, so read the key press from the terminal directly.
    with open('/dev/tty') as tty:
        print('Recording. Press Enter to stop.', flush=True)
        tty.readline()


def record(seconds):
    """Record one clip and return its path on the camera."""
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
        new_insv = sorted(f for f in set(cam.files_list()) - before if f.lower().endswith('.insv'))
    if len(new_insv) != 1:
        raise RuntimeError(f'expected one new .insv on the camera, found {new_insv}. '
                           'Split-lens modes (_00_ + _10_ files) are not supported by 00_convert_insv.py.')
    return new_insv[0]


def download(remote, partial):
    pi.set_log_level(pi.LogLevel.WARNING)
    shown = [-1]

    def progress(done, total):
        pct = done * 100 // max(total, 1)
        if pct >= shown[0] + 10:
            shown[0] = pct
            print(f'downloading {remote}: {pct}%', flush=True)

    with pi.Camera() as cam:
        cam.download_file(remote, partial, progress)


def download_with_retries(remote, local):
    """download_file has no timeout and stalled at 0 bytes right after a 60 s take, so watch it from outside.
    The retry loop also covers the SDK's discovery crash, which run_supervised handles for recording."""
    partial = local.with_name(local.name + '.part')
    ctx = multiprocessing.get_context('spawn')
    for attempt in range(1, DOWNLOAD_ATTEMPTS + 1):
        time.sleep(DOWNLOAD_SETTLE_S)
        proc = ctx.Process(target=download, args=(remote, str(partial)))
        proc.start()
        last_size, last_change = -1, time.monotonic()
        while proc.is_alive() and time.monotonic() - last_change < DOWNLOAD_STALL_S:
            size = partial.stat().st_size if partial.exists() else 0
            if size != last_size:
                last_size, last_change = size, time.monotonic()
            proc.join(1)
        if proc.is_alive():
            proc.kill()
            proc.join()
            print(f'download attempt {attempt} made no progress for {DOWNLOAD_STALL_S} s, retrying', flush=True)
        elif proc.exitcode == 0:
            partial.rename(local)
            print(f'saved {local}', flush=True)
            return
        else:
            print(f'download attempt {attempt} failed with exit code {proc.exitcode}, retrying', flush=True)
    raise SystemExit(f'could not download {remote}; it is still on the camera. Rerun with --download {remote}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir')
    ap.add_argument('--kind', choices=['demo', 'mapping', 'gripper_calibration'], default='demo')
    ap.add_argument('--seconds', type=float, default=None, help='Stop after this long instead of on Enter')
    ap.add_argument('--download', metavar='REMOTE', default=None,
                    help='Skip recording and fetch this camera path, e.g. /DCIM/Camera01/VID_..._00_008.insv')
    args = ap.parse_args()

    raw_videos = pathlib.Path(args.session_dir).expanduser().absolute() / 'raw_videos'
    out_dir = raw_videos / 'gripper_calibration' if args.kind == 'gripper_calibration' else raw_videos
    mapping_path = raw_videos / 'mapping.insv'
    if args.kind == 'mapping' and (mapping_path.exists() or raw_videos.joinpath('mapping.mp4').exists()):
        raise SystemExit(f'{mapping_path} already exists; a session has one mapping video.')
    remote = args.download or pi.run_supervised(record, args.seconds)
    local = mapping_path if args.kind == 'mapping' else out_dir / pathlib.PurePosixPath(remote).name
    local.parent.mkdir(parents=True, exist_ok=True)
    download_with_retries(remote, local)


if __name__ == '__main__':
    main()
