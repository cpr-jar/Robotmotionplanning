"""Build independent CMake consumers and verify their actual component/source sets.

Each case gets a fresh isolated configuration. Only pinned local source archives
are used; execution/planning/DXF need no Eigen/XML/YAML/Ruckig dependency.
"""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys

from check_layers import inspect

ROOT = Path(__file__).resolve().parents[2]
SOURCES = {
    'serial_kinematics': ['src/kinematics/serial_chain_kinematics.cpp'],
    'differential_kinematics': ['src/kinematics/inverse_differential.cpp'],
    'calibration': ['src/calibration/rigid_calibration.cpp'],
    'dxf_import': ['src/io/dxf_path_import.cpp'],
    'model_import': ['src/io/robot_model_import.cpp'],
    'program': ['src/program/program_import.cpp', 'src/program/prepared_robot_program.cpp',
                'src/program/program_planning.cpp', 'src/program/program_dxf.cpp',
                'src/program/program_simulation.cpp'],
}
CASES = ['execution', 'planning', 'serial_kinematics', 'serial_planning', 'differential_kinematics',
         'calibration', 'dxf_import', 'model_import', 'program', 'serial', 'following']


def run_case(case, build, cmake, sanitize):
    build.mkdir(parents=True, exist_ok=True)
    query = build / '.cmake/api/v1/query/codemodel-v2'
    query.parent.mkdir(parents=True, exist_ok=True)
    query.touch()
    log = build / 'validation.log'
    environment = dict(os.environ, ASAN_OPTIONS='detect_leaks=0', UBSAN_OPTIONS='halt_on_error=1')
    commands = [
        [cmake, '-S', str(ROOT / 'tests/integration/components'), '-B', str(build),
         '-DROBOT_MOTION_SOURCE_DIR=' + str(ROOT), '-DCASE=' + case,
         '-DCMAKE_BUILD_TYPE=' + ('Debug' if sanitize else 'Release'),
         '-DCMAKE_CXX_FLAGS_DEBUG=-O1 -g1', '-DMOTION_SANITIZE=' + ('ON' if sanitize else 'OFF')],
        [cmake, '--build', str(build), '--parallel', '2', '--config', 'Debug' if sanitize else 'Release'],
    ]
    with log.open('w') as output:
        for command in commands:
            subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, env=environment,
                           check=True, timeout=300)
        candidates = [build / 'component_consumer', build / 'component_consumer.exe',
                      build / ('Debug' if sanitize else 'Release') / 'component_consumer.exe']
        executable = next((p for p in candidates if p.is_file()), None)
        if executable is None:
            raise ValueError('Missing component consumer executable')
        subprocess.run([str(executable)], stdout=output, stderr=subprocess.STDOUT,
                       env=environment, check=True, timeout=180)
    policy = json.loads((ROOT / 'docs/architecture/layers.json').read_text())
    dependencies = inspect(ROOT, policy)['components']
    pending = ['planning', 'serial_kinematics'] if case == 'serial_planning' else [case]
    expected = set()
    while pending:
        name = pending.pop()
        if name not in expected:
            expected.add(name)
            pending.extend(dependencies[name])
    present = set((build / 'components.txt').read_text().strip().split(';'))
    if present != expected:
        raise ValueError(f'Unexpected exported components: {present ^ expected}')
    reply = build / '.cmake/api/v1/reply'
    index = json.loads(sorted(reply.glob('index-*.json'))[-1].read_text())
    model = json.loads((reply / index['reply']['codemodel-v2']['jsonFile']).read_text())
    cpp_sources, archives = set(), set()
    for target_ref in model['configurations'][0]['targets']:
        if not target_ref['name'].startswith('robot_motion_'):
            continue
        target = json.loads((reply / target_ref['jsonFile']).read_text())
        if target['type'] == 'STATIC_LIBRARY':
            archives.add(target['name'].removeprefix('robot_motion_'))
        for source in target.get('sources', []):
            path = Path(source['path'])
            if not path.is_absolute():
                path = Path(model['paths']['source']) / path
            if path.suffix == '.cpp':
                cpp_sources.add(path.resolve().relative_to(ROOT).as_posix())
    wanted_sources = {source for component in expected for source in SOURCES.get(component, [])}
    wanted_archives = expected.intersection(set(SOURCES) | {'serial'})
    if cpp_sources != wanted_sources or archives != wanted_archives:
        raise ValueError(f'Unexpected source/archive set: {cpp_sources ^ wanted_sources}, {archives ^ wanted_archives}')
    # The legacy archive must contain real implementation objects, not just an empty anchor.
    if case == 'serial':
        archive = next(build.rglob('*robot_motion_serial.a'), None)
        if archive is not None and shutil.which('ar'):
            members = subprocess.check_output(['ar', 't', str(archive)], text=True).splitlines()
            if len(members) != len(wanted_sources):
                raise ValueError('Historical archive no longer contains every implementation object')
    if case in ('execution', 'planning', 'dxf_import') and any(build.glob('_deps/*-src')):
        raise ValueError('Pure runtime/planning/DXF fetched unrequested external sources')
    report = {'case': case, 'sanitize': sanitize, 'components': sorted(present),
              'cpp_sources': sorted(cpp_sources), 'archives': sorted(archives), 'passed': True}
    (build / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, action='append')
    parser.add_argument('--cmake', default='cmake')
    parser.add_argument('--build-root', type=Path, default=ROOT / 'build-components')
    parser.add_argument('--sanitize', action='store_true')
    options = parser.parse_args()
    cmake = shutil.which(options.cmake)
    if not cmake:
        parser.error('CMake executable not found')
    for case in options.case or CASES:
        build = options.build_root.resolve() / case
        try:
            report = run_case(case, build, cmake, options.sanitize)
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            print(f'{case} failed: {error}; see {build / "validation.log"}', file=sys.stderr)
            return 1
        print(f"{case}: passed, {len(report['cpp_sources'])} implementation files, "
              f"{len(report['archives'])} motion archives", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
