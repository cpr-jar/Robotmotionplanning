"""Check maintained include paths, compatibility headers, diagram declarations and local doc links."""
from pathlib import Path
import json
import re
import sys
from render_diagrams import validate_node

ROOT = Path(__file__).resolve().parents[2]
errors = []
manifest = json.loads((ROOT / 'docs/architecture/file_migration.json').read_text())
for previous, current in manifest.items():
    destination = ROOT / current
    if not destination.is_file():
        errors.append(f'Missing migration destination: {current}')
    if previous.startswith('include/motion/'):
        compatibility = ROOT / previous
        expected_include = '#include "' + current.removeprefix('include/') + '"'
        if not compatibility.is_file() or expected_include not in compatibility.read_text():
            errors.append(f'Invalid compatibility header: {previous}')

for directory in ['src', 'examples', 'include/motion']:
    for source in (ROOT / directory).rglob('*'):
        if source.suffix not in ('.cpp', '.hpp'):
            continue
        if source.parent == ROOT / 'include/motion':
            continue  # forwarding headers are deliberately retained here
        for include in re.findall(r'#include\s+"(motion/[^"]+)"', source.read_text()):
            if not (ROOT / 'include' / include).is_file():
                errors.append(f'{source.relative_to(ROOT)}: missing include {include}')
            if include.count('/') == 1:
                errors.append(f'{source.relative_to(ROOT)}: use canonical include instead of {include}')

for document in [ROOT / 'README.md', *sorted((ROOT / 'docs').rglob('*.md'))]:
    for link in re.findall(r'\]\(([^)]+)\)', document.read_text()):
        if '://' in link or link.startswith(('#', '/', 'mailto:')):
            continue
        path = link.split('#', 1)[0]
        if path and not (document.parent / path).exists():
            errors.append(f'{document.relative_to(ROOT)}: missing local link {link}')

specs = json.loads((ROOT / 'docs/architecture/diagrams/class_diagrams.json').read_text())
for diagram in specs:
    for node in diagram['nodes']:
        try:
            validate_node(node)
        except ValueError as error:
            errors.append(str(error))
    for extension in ['dot', 'svg']:
        if not (ROOT / 'docs/architecture/diagrams' / (diagram['id'] + '.' + extension)).is_file():
            errors.append(f'Missing {extension} for diagram {diagram["id"]}')
if errors:
    print('\n'.join(errors), file=sys.stderr)
    sys.exit(1)
print(f'Structure checked: {len(manifest)} migration entries, {len(specs)} UML diagrams, local documentation links')
