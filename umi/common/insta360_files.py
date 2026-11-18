"""Insta360 camera file naming and session folder roles, shared by the camera-side and pipeline-side code."""
import re

# VID_20261119_032839_10_013.insv: date, time, lens digit, proxy digit, take index.
CAMERA_FILE_RE = re.compile(
    r'^(?P<kind>VID|LRV)_(?P<date>\d{8})_(?P<time>\d{6})_(?P<lens>\d)(?P<proxy>\d)_(?P<index>\d+)\.(?:insv|lrv)$',
    re.IGNORECASE)
# A take inside raw_videos/mapping/ converts to raw_videos/mapping.mp4, which 00_process_videos expects.
MAPPING_DIR = 'mapping'
GRIPPER_CALIBRATION_DIR = 'gripper_calibration'
