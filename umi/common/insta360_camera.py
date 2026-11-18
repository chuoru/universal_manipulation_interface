"""
Camera-side helpers for scripts/insta360_record.py and scripts/insta360_import.py.
Needs pyinsta360 (Python 3.10+), so the umi environment does not import this module.
"""
import collections
import dataclasses
import datetime
import multiprocessing
import pathlib
import time
from typing import Dict, List, Optional

import pyinsta360 as pi

from umi.common.insta360_files import CAMERA_FILE_RE, GRIPPER_CALIBRATION_DIR, MAPPING_DIR

ROLES = ('demo', 'mapping', 'gripper_calibration')
DOWNLOAD_SETTLE_S = 5
# The first download after a take often sits at 0 bytes forever; a healthy one writes ~18 MB/s.
DOWNLOAD_STALL_S = 20
DOWNLOAD_ATTEMPTS = 4


@dataclasses.dataclass(frozen=True)
class CameraTake:
    """One take on the camera card: its lens files and its LRV proxy."""
    index: str
    started: datetime.datetime
    vids: Dict[str, str]
    lrv: Optional[str]

    @property
    def split(self) -> bool:
        return '1' in self.vids

    def remotes(self, lens_idx: int) -> List[str]:
        """Camera paths 00_convert_insv.py needs: the packed file, or the lens file plus the LRV."""
        if not self.split:
            return [self.vids['0']]
        if str(lens_idx) not in self.vids or self.lrv is None:
            raise ValueError(f'take {self.index} is split-lens but lacks the lens {lens_idx} file or its LRV')
        return [self.vids[str(lens_idx)], self.lrv]


def group_takes(remote_files) -> List[CameraTake]:
    parts = collections.defaultdict(dict)
    for remote in remote_files:
        m = CAMERA_FILE_RE.match(pathlib.PurePosixPath(remote).name)
        if m is None:
            continue
        key = (m['date'], m['time'], m['index'])
        parts[key].setdefault('vids', {})
        if m['kind'].upper() == 'VID':
            parts[key]['vids'][m['lens']] = remote
        else:
            parts[key]['lrv'] = remote
    takes = [CameraTake(index=index, started=datetime.datetime.strptime(date + t, '%Y%m%d%H%M%S'),
                        vids=p['vids'], lrv=p.get('lrv'))
             for (date, t, index), p in parts.items() if p['vids']]
    return sorted(takes, key=lambda take: take.started)


def role_dir(session: pathlib.Path, role: str) -> pathlib.Path:
    raw = session / 'raw_videos'
    return {'mapping': raw / MAPPING_DIR, 'gripper_calibration': raw / GRIPPER_CALIBRATION_DIR, 'demo': raw}[role]


def session_file_names(session: pathlib.Path) -> set:
    """Names of camera files already in the session, waiting in raw_videos or converted into raw_insv."""
    return {p.name for p in session.rglob('*') if CAMERA_FILE_RE.match(p.name)}


def mapping_exists(session: pathlib.Path) -> bool:
    raw = session / 'raw_videos'
    return (raw / 'mapping.mp4').exists() or (raw / 'mapping.mp4').is_symlink() or \
        any((raw / MAPPING_DIR).glob('*')) or any((session / 'raw_insv' / 'raw_videos' / MAPPING_DIR).glob('*'))


def list_camera_files() -> List[str]:
    pi.set_log_level(pi.LogLevel.WARNING)
    with pi.Camera() as cam:
        return cam.files_list()


def fetch_take(take: CameraTake, role: str, session: pathlib.Path, lens_idx: int) -> List[pathlib.Path]:
    """Download what the take needs into its role folder; files already in the session are skipped."""
    have = session_file_names(session)
    saved = []
    for remote in take.remotes(lens_idx):
        name = pathlib.PurePosixPath(remote).name
        if name in have:
            print(f'{name} already in session, skipping', flush=True)
            continue
        local = role_dir(session, role) / name
        local.parent.mkdir(parents=True, exist_ok=True)
        download_with_retries(remote, local)
        saved.append(local)
    return saved


def _download(remote: str, partial: str):
    pi.set_log_level(pi.LogLevel.WARNING)
    shown = [-1]

    def progress(done, total):
        pct = done * 100 // max(total, 1)
        if pct >= shown[0] + 10:
            shown[0] = pct
            print(f'downloading {remote}: {pct}%', flush=True)

    with pi.Camera() as cam:
        cam.download_file(remote, partial, progress)


def download_with_retries(remote: str, local: pathlib.Path):
    """download_file has no timeout and stalls at 0 bytes right after a take, so watch the file from outside.
    The retry loop also covers the SDK's discovery crash."""
    partial = local.with_name(local.name + '.part')
    ctx = multiprocessing.get_context('spawn')
    for attempt in range(1, DOWNLOAD_ATTEMPTS + 1):
        time.sleep(DOWNLOAD_SETTLE_S)
        proc = ctx.Process(target=_download, args=(remote, str(partial)))
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
    raise SystemExit(f'could not download {remote}; it is still on the camera. Rerun the import for this take.')
