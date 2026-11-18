"""
Import takes recorded on an Insta360 (with its own button, no PC attached) into a UMI session.

Run with the Python that has pyinsta360 installed (Python 3.10+). The camera must be in
Android USB mode. List the takes on the card, then pick them by index and role:

python scripts/insta360_import.py data/session1 --list
python scripts/insta360_import.py data/session1 --mapping 13 --gripper-calibration 11 --demo 12 14-20

Only what 00_convert_insv.py needs is downloaded: the .insv of a packed take, or the
chosen lens file and the LRV proxy of a split take. Takes land where insta360_record.py
puts them, files already in the session are skipped, and nothing is deleted from the camera.
"""
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse
import pathlib

import pyinsta360 as pi

from umi.common.insta360_camera import fetch_take, group_takes, list_camera_files, mapping_exists, session_file_names


def parse_indices(specs):
    """'12', '014', '14-20' -> take indices as ints."""
    out = []
    for spec in specs or []:
        lo, _, hi = spec.partition('-')
        out.extend(range(int(lo), int(hi or lo) + 1))
    return out


def print_takes(takes, session, lens_idx):
    have = session_file_names(session)
    print(f'{"take":>5}  {"started (camera time)":21}  {"layout":7}  status')
    for take in takes:
        try:
            names = [pathlib.PurePosixPath(r).name for r in take.remotes(lens_idx)]
            status = 'in session' if all(n in have for n in names) else ''
        except ValueError as e:
            status = f'unusable: {e}'
        print(f'{int(take.index):>5}  {take.started:%Y-%m-%d %H:%M:%S}    {"split" if take.split else "packed":7}  {status}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir')
    ap.add_argument('--list', action='store_true', help='List the takes on the camera and exit')
    ap.add_argument('--mapping', metavar='TAKE', help='Take index to use as the mapping video')
    ap.add_argument('--gripper-calibration', metavar='TAKE', nargs='+', help='Take indices for gripper calibration')
    ap.add_argument('--demo', metavar='TAKE', nargs='+', help='Take indices or ranges (14-20) for demos')
    ap.add_argument('--lens', type=int, default=1, help='Lens the pipeline keeps; picks the file of a split take')
    args = ap.parse_args()

    session = pathlib.Path(args.session_dir).expanduser().absolute()
    takes = group_takes(pi.run_supervised(list_camera_files))
    if args.list:
        print_takes(takes, session, args.lens)
        return

    roles = {}
    for role, indices in (('mapping', parse_indices([args.mapping] if args.mapping else [])),
                          ('gripper_calibration', parse_indices(args.gripper_calibration)),
                          ('demo', parse_indices(args.demo))):
        for index in indices:
            if index in roles:
                raise SystemExit(f'take {index} is given two roles: {roles[index]} and {role}')
            roles[index] = role
    if not roles:
        raise SystemExit('nothing to import; pass --list, --mapping, --gripper-calibration or --demo')
    if 'mapping' in roles.values() and mapping_exists(session):
        raise SystemExit(f'{session} already has a mapping take; a session has one mapping video.')

    by_index = {int(take.index): take for take in takes}
    missing = sorted(set(roles) - set(by_index))
    if missing:
        raise SystemExit(f'takes not on the camera: {missing}. Run with --list.')
    for index, role in sorted(roles.items()):
        print(f'take {index} -> {role}', flush=True)
        fetch_take(by_index[index], role, session, args.lens)


if __name__ == '__main__':
    main()
