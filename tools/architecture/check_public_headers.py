"""Compile each canonical public header in isolation (optional developer check)."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OPTIONAL_RUCKIG = {
    'planning/joint_stop.hpp', 'planning/ruckig_joint_generator.hpp',
    'execution/stop_schedules.hpp', 'tracking/conveyor_following.hpp',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default='c++')
    parser.add_argument('--ruckig-include', type=Path, help='directory containing ruckig/ruckig.hpp; enables optional headers')
    parser.add_argument('--with-serial', action='store_true')
    args = parser.parse_args()
    flags = [args.compiler, '-std=c++20', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I' + str(ROOT / 'include')]
    if args.with_serial:
        flags.append('-DMOTION_WITH_SERIAL=1')
    if args.ruckig_include:
        flags += ['-DMOTION_WITH_RUCKIG=1', '-isystem', str(args.ruckig_include)]
    headers = []
    for header in sorted((ROOT / 'include/motion').glob('*/*.hpp')):
        relative = header.relative_to(ROOT / 'include/motion').as_posix()
        if relative in OPTIONAL_RUCKIG and not args.ruckig_include:
            continue
        headers.append(header.relative_to(ROOT / 'include').as_posix())

    def compile_header(header):
        result = subprocess.run(flags + ['-x', 'c++', '-fsyntax-only', '-'],
                                input='#include <' + header + '>\n', text=True, capture_output=True)
        return header, result

    with ThreadPoolExecutor(max_workers=2) as workers:
        results = list(workers.map(compile_header, headers))
    failed = False
    for header, result in results:
        if result.returncode:
            failed = True
            print(header + '\n' + result.stderr, file=sys.stderr)
    if failed:
        return 1
    print(f'Compiled {len(headers)} canonical headers individually with {args.compiler}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
