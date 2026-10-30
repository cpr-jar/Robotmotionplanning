"""Check conservative C++ include dependencies and the pure runtime header closure.

Both conditional branches count. Two declared compatibility facades remain
available to clients, but production code must use their canonical contracts.
"""
from pathlib import Path
import argparse
import json
import re
import shlex
import sys

ROOT = Path(__file__).resolve().parents[2]
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^>"\n]+)[>"]', re.MULTILINE)


def cycles(graph):
    visited, active, found = set(), [], []

    def visit(node):
        if node in active:
            found.append(active[active.index(node):] + [node])
            return
        if node in visited:
            return
        active.append(node)
        for child in sorted(graph.get(node, [])):
            visit(child)
        active.pop()
        visited.add(node)

    for node in sorted(graph):
        visit(node)
    return found


def inspect(root, policy):
    base = root / 'include/motion'
    allowed = policy['allowed_dependencies']
    facades = policy.get('compatibility_headers', {})
    errors = []
    files = sorted(p for p in base.rglob('*.hpp') if len(p.relative_to(base).parts) > 1)
    files += sorted(p for p in (root / 'src').rglob('*') if p.suffix in ('.cpp', '.hpp'))
    graph = {name: set() for name in allowed}
    includes = {}
    for previous, canonical in facades.items():
        path = base / previous
        if not path.is_file() or f'#include "motion/{canonical}"' not in path.read_text():
            errors.append(f'Invalid compatibility facade: {previous}')
    for path in files:
        if path.is_relative_to(base):
            relative = path.relative_to(base)
            if relative.as_posix() in facades:
                continue
        else:
            relative = path.relative_to(root / 'src')
        module = relative.parts[0]
        if module not in allowed:
            errors.append(f'Unknown layer: {path.relative_to(root)}')
            continue
        key = path.relative_to(root).as_posix()
        includes[key] = set()
        for name in INCLUDE.findall(path.read_text()):
            if name.startswith('motion/'):
                header = name.removeprefix('motion/')
                target = root / 'include' / name
                if header in facades or '/' not in header:
                    errors.append(f'{key}: production include of compatibility header {name}')
                    continue
                if not target.is_file():
                    errors.append(f'{key}: missing include {name}')
                    continue
                dependency = Path(header).parts[0]
                if dependency != module:
                    graph[module].add(dependency)
                    if dependency not in allowed[module]:
                        errors.append(f'Forbidden {module} -> {dependency}: {key} -> {name}')
                includes[key].add(target.relative_to(root).as_posix())
            elif (path.parent / name).is_file():
                includes[key].add((path.parent / name).relative_to(root).as_posix())
    for label, dependencies in [('declared layer', allowed), ('actual layer', graph), ('file', includes)]:
        for cycle in cycles(dependencies):
            errors.append(f'{label} cycle: ' + ' -> '.join(cycle))
    runtime = {}
    for entry in policy.get('runtime_entries', []):
        pending, closure = ['include/motion/' + entry], set()
        if pending[0] not in includes:
            errors.append(f'Missing runtime entry: {entry}')
        while pending:
            key = pending.pop()
            if key in closure:
                continue
            closure.add(key)
            pending.extend(includes.get(key, []))
        layers = sorted({Path(p).parts[2] for p in closure})
        runtime[entry] = {'layers': layers, 'headers': sorted(closure)}
        for layer in layers:
            if layer not in policy['runtime_allowed_layers']:
                errors.append(f'Runtime entry {entry} includes forbidden layer {layer}')
    components = {}
    if policy.get('component_file'):
        text = (root / policy['component_file']).read_text()
        for name, values in re.findall(r'set\(motion_deps_(\w+)\s+([^)]*)\)', text):
            components[name] = [value for value in shlex.split(values) if value]
        known = re.search(r'set\(MOTION_KNOWN_COMPONENTS\s+([^)]*)\)', text)
        if not known or set(shlex.split(known[1])) != set(components):
            errors.append('CMake component registry/dependency declarations differ')
        if not components:
            errors.append('Missing CMake component declarations')
        for name, dependencies in components.items():
            for dependency in dependencies:
                if dependency not in components:
                    errors.append(f'Unknown CMake dependency {name} -> {dependency}')
        for cycle in cycles(components):
            errors.append('CMake component cycle: ' + ' -> '.join(cycle))
    return {'layers': {k: sorted(v) for k, v in sorted(graph.items())}, 'components': components,
            'canonical_files': len(includes), 'compatibility_facades': sorted(facades),
            'runtime': runtime, 'errors': errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', type=Path, help='Write the measured graph and closures')
    args = parser.parse_args()
    policy = json.loads((ROOT / 'docs/architecture/layers.json').read_text())
    report = inspect(ROOT, policy)
    if args.json:
        args.json.write_text(json.dumps(report, indent=2) + '\n')
    if report['errors']:
        print('\n'.join(report['errors']), file=sys.stderr)
        return 1
    print(f"Layers checked: {len(report['layers'])} layers, {len(report['components'])} components, "
          f"{report['canonical_files']} files; "
          f"no file/module cycles, {len(report['runtime'])} pure runtime closures")
    return 0


if __name__ == '__main__':
    sys.exit(main())
