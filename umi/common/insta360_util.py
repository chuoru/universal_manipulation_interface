"""
Read Insta360 .insv files (dual-fisheye video + IMU trailer) into the
artifacts the UMI SLAM pipeline consumes: a single-lens mp4, an
OpenICC/GPMF-style imu_data.json, and per-lens fisheye intrinsics.
"""
import dataclasses
import datetime
import json
import pathlib
import subprocess
from typing import Dict, Tuple

import numpy as np
import telemetry_parser

DEG_TO_RAD = np.pi / 180.0
# offset_v3 layout: [n_lens, then per lens 19 values]
# xi fx fy cx cy yaw pitch roll tx ty tz k1 k2 k3 p1 p2 sensor_w sensor_h lens_type
OFFSET_V3_STRIDE = 19
# Insta360 records its factory calibration up to this angle off the optical axis.
FOV_LIMIT_RAD = 100.0 * DEG_TO_RAD


@dataclasses.dataclass(frozen=True)
class MeiLens:
    """Unified (Mei) camera model in pixels of one cropped lens image."""
    xi: float
    fx: float
    fy: float
    cx: float
    cy: float
    k: Tuple[float, float, float]
    p: Tuple[float, float]
    size: int

    def project_radius(self, theta: np.ndarray) -> np.ndarray:
        """Distorted radius, in normalized (focal=1) units, of a ray at angle theta."""
        x = np.sin(theta) / (np.cos(theta) + self.xi)
        r2 = x * x
        k1, k2, k3 = self.k
        return x * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3)


@dataclasses.dataclass(frozen=True)
class KannalaBrandt:
    """OpenCV fisheye / ORB_SLAM3 KannalaBrandt8 model."""
    fx: float
    fy: float
    cx: float
    cy: float
    k: Tuple[float, float, float, float]
    width: int
    height: int
    max_fit_error_px: float

    def scaled(self, width: int, height: int) -> 'KannalaBrandt':
        sx, sy = width / self.width, height / self.height
        return dataclasses.replace(
            self, fx=self.fx * sx, fy=self.fy * sy, cx=self.cx * sx, cy=self.cy * sy,
            width=width, height=height, max_fit_error_px=self.max_fit_error_px * sx)

    def radius_px(self, theta: float) -> float:
        k1, k2, k3, k4 = self.k
        return self.fx * theta * (1 + k1 * theta ** 2 + k2 * theta ** 4 + k3 * theta ** 6 + k4 * theta ** 8)


@dataclasses.dataclass(frozen=True)
class InsvClip:
    path: pathlib.Path
    serial: str
    camera_model: str
    start_datetime: datetime.datetime
    fps: float
    n_frames: int
    width: int
    height: int
    imu_t_ms: np.ndarray
    accl: np.ndarray
    gyro: np.ndarray
    lenses: Tuple[MeiLens, ...]


def _ffprobe_video(path: pathlib.Path) -> dict:
    out = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v',
         '-show_entries', 'stream=width,height,avg_frame_rate,nb_frames:stream_tags=creation_time',
         '-of', 'json', str(path)],
        check=True, capture_output=True, text=True).stdout
    streams = json.loads(out)['streams']
    if len(streams) != 1:
        raise ValueError(f'{path.name}: expected one packed dual-fisheye video stream, found {len(streams)}')
    return streams[0]


def _parse_lenses(metadata: dict, width: int, height: int) -> Tuple[MeiLens, ...]:
    v3 = metadata['offset_v3']
    n_lens = int(v3[0])
    lens_size = width // n_lens
    if lens_size != height:
        raise ValueError(f'expected {n_lens} square lenses side by side, got {width}x{height}')
    lenses = []
    for i in range(n_lens):
        xi, fx, fy, cx, cy = v3[1 + i * OFFSET_V3_STRIDE: 6 + i * OFFSET_V3_STRIDE]
        k = tuple(v3[12 + i * OFFSET_V3_STRIDE: 15 + i * OFFSET_V3_STRIDE])
        p = tuple(v3[15 + i * OFFSET_V3_STRIDE: 17 + i * OFFSET_V3_STRIDE])
        sensor_w = v3[17 + i * OFFSET_V3_STRIDE]
        s = width / sensor_w
        lenses.append(MeiLens(
            xi=xi, fx=fx * s, fy=fy * s, cx=cx * s - i * lens_size, cy=cy * s,
            k=k, p=p, size=lens_size))
    return tuple(lenses)


def read_insv(path) -> InsvClip:
    path = pathlib.Path(path)
    parser = telemetry_parser.Parser(str(path))
    metadata = next(g['Default']['Metadata'] for g in parser.telemetry() if 'Default' in g)
    video = _ffprobe_video(path)
    width, height = int(video['width']), int(video['height'])
    num, den = (int(x) for x in video['avg_frame_rate'].split('/'))

    # normalized_imu: timestamps in ms relative to the first video frame,
    # accl in m/s^2, gyro in deg/s. Samples before the first frame are pre-roll.
    imu = parser.normalized_imu()
    t_ms = np.array([s['timestamp_ms'] for s in imu])
    keep = t_ms >= 0
    accl = np.array([s['accl'] for s in imu])[keep]
    gyro = np.array([s['gyro'] for s in imu])[keep] * DEG_TO_RAD

    created = video['tags']['creation_time']
    start = datetime.datetime.strptime(created, r'%Y-%m-%dT%H:%M:%S.%fZ')
    return InsvClip(
        path=path,
        serial=metadata['serial_number'],
        camera_model=metadata['camera_type'],
        start_datetime=start,
        fps=num / den,
        n_frames=int(video['nb_frames']),
        width=width, height=height,
        imu_t_ms=t_ms[keep], accl=accl, gyro=gyro,
        lenses=_parse_lenses(metadata, width, height))


def fit_kannala_brandt(lens: MeiLens, max_theta: float = FOV_LIMIT_RAD) -> KannalaBrandt:
    """Least-squares fit r = f*(t + k1 t^3 + k2 t^5 + k3 t^7 + k4 t^9) to the Mei model."""
    theta = np.linspace(1e-4, max_theta, 2000)
    r = lens.project_radius(theta)
    basis = np.stack([theta ** (2 * i + 1) for i in range(5)], axis=1)
    coef, *_ = np.linalg.lstsq(basis, r, rcond=None)
    fit_err = np.abs(basis @ coef - r)
    f_scale = coef[0]
    k = tuple(float(c / f_scale) for c in coef[1:])
    return KannalaBrandt(
        fx=lens.fx * f_scale, fy=lens.fy * f_scale, cx=lens.cx, cy=lens.cy, k=k,
        width=lens.size, height=lens.size,
        max_fit_error_px=float(fit_err.max() * max(lens.fx, lens.fy)))


def imu_to_gpmf_json(clip: InsvClip) -> dict:
    """The subset of the OpenICC GoPro telemetry schema that gopro_slam reads."""
    def samples(values):
        return [{'value': v.tolist(), 'cts': float(t)} for t, v in zip(clip.imu_t_ms, values)]
    frame_cts = np.arange(clip.n_frames) * 1000.0 / clip.fps
    return {
        '1': {'streams': {
            'ACCL': {'samples': samples(clip.accl)},
            'GYRO': {'samples': samples(clip.gyro)},
            'CORI': {'samples': [{'value': [1.0, 0.0, 0.0, 0.0], 'cts': float(t)} for t in frame_cts]},
        }},
        'frames/second': clip.fps,
        'camera_model': clip.camera_model,
        'camera_serial': clip.serial,
    }


def write_lens_mp4(clip: InsvClip, lens_idx: int, out_path, crf: int = 18) -> None:
    """Crop one lens, keep every frame, and tag serial and start timecode the way GoPro files carry them."""
    size = clip.lenses[lens_idx].size
    start = clip.start_datetime
    frame = int(round(start.microsecond * 1e-6 * clip.fps))
    timecode = f'{start:%H:%M:%S}:{frame:02d}'
    subprocess.run([
        'ffmpeg', '-v', 'error', '-y', '-i', str(clip.path),
        '-map', '0:v:0', '-vf', f'crop={size}:{size}:{lens_idx * size}:0',
        '-fps_mode', 'passthrough', '-c:v', 'libx264', '-preset', 'fast', '-crf', str(crf),
        '-pix_fmt', 'yuv420p', '-timecode', timecode,
        '-movflags', 'use_metadata_tags',
        '-metadata', f'creation_time={start:%Y-%m-%dT%H:%M:%S.%fZ}',
        '-metadata', f'CameraSerialNumber={clip.serial}',
        '-metadata', f'Model={clip.camera_model}',
        str(out_path)], check=True)


def lens_mask(kb: KannalaBrandt, max_theta: float) -> np.ndarray:
    """255 outside the cone of max_theta, the convention gopro_slam's --mask_img expects."""
    yy, xx = np.mgrid[:kb.height, :kb.width]
    r = np.hypot(xx - kb.cx, (yy - kb.cy) * kb.fx / kb.fy)
    return np.where(r > kb.radius_px(max_theta), 255, 0).astype(np.uint8)


def orbslam_settings_yaml(kb: KannalaBrandt, fps: float, T_b_c: np.ndarray, imu_hz: float) -> str:
    data = ', '.join(f'{v:.8f}' for v in T_b_c.reshape(-1))
    k1, k2, k3, k4 = kb.k
    return f"""%YAML:1.0
File.version: "1.0"
Camera.type: "KannalaBrandt8"
Camera1.fx: {kb.fx:.6f}
Camera1.fy: {kb.fy:.6f}
Camera1.cx: {kb.cx:.6f}
Camera1.cy: {kb.cy:.6f}
Camera1.k1: {k1:.10f}
Camera1.k2: {k2:.10f}
Camera1.k3: {k3:.10f}
Camera1.k4: {k4:.10f}
Camera.width: {kb.width}
Camera.height: {kb.height}
Camera.fps: {int(round(fps))}
Camera.RGB: 1
IMU.T_b_c1: !!opencv-matrix
    rows: 4
    cols: 4
    dt: f
    data: [{data}]
# Allan deviation of X3 IAQEB2606EEWG9 at rest, inflated (noise x5, walk x8) for handheld
# vibration. From RoboManipBaselines envs/real/insta360_bridge/Insta360_X4.yaml.
IMU.NoiseGyro: 8.429130e-04
IMU.NoiseAcc: 2.011808e-02
IMU.GyroWalk: 6.339842e-04
IMU.AccWalk: 7.626927e-03
IMU.Frequency: {imu_hz:.1f}
ORBextractor.nFeatures: 1250
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
System.thFarPoints: 20.0
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
Viewer.imageViewScale: 1.0
"""


def openicc_intrinsics_json(kb: KannalaBrandt, fps: float) -> Dict:
    k1, k2, k3, k4 = kb.k
    return {
        'final_reproj_error': kb.max_fit_error_px,
        'fps': fps,
        'image_height': kb.height,
        'image_width': kb.width,
        'intrinsic_type': 'FISHEYE',
        'intrinsics': {
            'aspect_ratio': kb.fy / kb.fx,
            'focal_length': kb.fx,
            'principal_pt_x': kb.cx,
            'principal_pt_y': kb.cy,
            'radial_distortion_1': k1,
            'radial_distortion_2': k2,
            'radial_distortion_3': k3,
            'radial_distortion_4': k4,
            'skew': 0.0,
        },
        'nr_calib_images': 0,
        'stabelized': False,
        'source': 'Insta360 factory Mei calibration, refit to Kannala-Brandt',
    }
