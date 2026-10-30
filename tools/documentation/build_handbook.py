"""Build a self-contained integration handbook from Markdown and inline UML SVGs.

Install tools/requirements-docs.txt for this optional developer operation.
No external scripts, stylesheets or images are loaded by the generated HTML.
"""
from pathlib import Path
from html import escape
from html.parser import HTMLParser
import argparse
import os
import re
import sys
import markdown
from markdown.extensions.toc import slugify_unicode

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'docs/handbook.html'
CHAPTERS = [
    'docs/README.md',
    'docs/tutorials/motion_planning_basics.md',
    'docs/integration/controller_integration.md',
    'docs/integration/interface_contracts.md',
    'docs/integration/live_motion_session.md',
    'docs/architecture/class_diagrams.md',
    'docs/architecture/runtime_flows.md',
    'docs/architecture/module_catalog.md',
    'docs/development/naming_and_migration.md',
    'docs/integration/acceptance_plan.md',
    'docs/formats/robot_model_yaml.md',
    'docs/formats/robot_program_yaml.md',
    'docs/formats/dxf_program_command.md',
    'docs/testing/validation.md',
]
IDENTIFIERS = {(ROOT / path).resolve(): 'chapter-' + Path(path).stem.lower() for path in CHAPTERS}


def chapter_html(path):
    source = ROOT / path
    chapter_id = IDENTIFIERS[source.resolve()]
    content = source.read_text()

    def image(match):
        title, target = match.groups()
        svg_file = (source.parent / target).resolve()
        if svg_file.suffix != '.svg' or not svg_file.is_relative_to(ROOT):
            raise ValueError(f'Only local SVG diagrams are supported: {target}')
        svg = svg_file.read_text()
        svg = svg[svg.index('<svg'):]
        # Namespace complete SVGs, including reusable paths and plot clip paths.
        # Matplotlib SVGs use both xlink:href and url(#id), unlike simple Graphviz nodes.
        prefix = chapter_id + '--figure-' + svg_file.stem + '-'
        identifiers = {name: prefix + name for name in re.findall(r'\bid="([^"]+)"', svg)}
        svg = re.sub(r'\bid="([^"]+)"', lambda m: 'id="' + identifiers[m[1]] + '"', svg)
        svg = re.sub(r'\bhref="#([^"]+)"',
                     lambda m: 'href="#' + identifiers.get(m[1], m[1]) + '"', svg)
        svg = re.sub(r'url\(#([^)]+)\)',
                     lambda m: 'url(#' + identifiers.get(m[1], m[1]) + ')', svg)
        return '<figure>' + svg + '<figcaption>' + escape(title) + '</figcaption></figure>'

    content = re.sub(r'!\[([^]]*)\]\(([^)]+)\)', image, content)

    def link(match):
        url = match[1]
        if '://' in url or url.startswith(('#', '/', 'mailto:')):
            return match[0]
        target, separator, fragment = url.partition('#')
        resolved = (source.parent / target).resolve()
        if resolved in IDENTIFIERS:
            new_url = '#' + IDENTIFIERS[resolved] + ('--' + fragment if separator else '')
        else:
            new_url = os.path.relpath(resolved, OUTPUT.parent) + (separator + fragment if separator else '')
        return '](' + new_url + ')'

    content = re.sub(r'\]\(([^)]+)\)', link, content)
    rendered = markdown.markdown(content, extensions=['tables', 'fenced_code', 'toc'],
                                 extension_configs={'toc': {'slugify': slugify_unicode}})
    rendered = re.sub(r'\bid="([^"]+)"',
                      lambda m: 'id="' + (m[1] if m[1].startswith(chapter_id + '--figure-')
                                          else chapter_id + '--' + m[1]) + '"', rendered)
    rendered = re.sub(r'href="#(?!chapter-)([^"]+)"', lambda m: 'href="#' + chapter_id + '--' + m[1] + '"', rendered)
    return '<section id="' + chapter_id + '">' + rendered + '</section>'


class AnchorCheck(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.targets = set()

    def handle_starttag(self, tag, attributes):
        attributes = dict(attributes)
        if 'id' in attributes:
            if attributes['id'] in self.ids:
                raise ValueError('Duplicate HTML ID: ' + attributes['id'])
            self.ids.add(attributes['id'])
        for key in ('href', 'xlink:href'):
            if attributes.get(key, '').startswith('#'):
                self.targets.add(attributes[key][1:])
        for value in attributes.values():
            self.targets.update(re.findall(r'url\(#([^)]+)\)', value))


def build():
    navigation = []
    sections = []
    for path in CHAPTERS:
        title = next(line[2:] for line in (ROOT / path).read_text().splitlines() if line.startswith('# '))
        navigation.append('<li><a href="#' + IDENTIFIERS[(ROOT / path).resolve()] + '">' + escape(title) + '</a></li>')
        sections.append(chapter_html(path))
    document = '''<!doctype html>
<html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Motion C++ 0.13.0 — Integrationshandbuch</title>
<style>
body{font:16px/1.6 system-ui,sans-serif;color:#172033;background:#fff;margin:0 auto;max-width:1100px;padding:24px}
h1,h2,h3{line-height:1.25;color:#102a43}h1{margin-top:1.5em}h2{margin-top:1.8em}
a{color:#165ca3}nav{padding:16px 24px;background:#eff6ff;border-radius:8px}section{border-top:1px solid #cbd5e1;margin-top:48px}
table{border-collapse:collapse;width:100%;margin:20px 0}th,td{border:1px solid #cbd5e1;text-align:left;padding:8px;vertical-align:top}th{background:#eff6ff}
code{font:0.9em ui-monospace,monospace;background:#f1f5f9;padding:2px 4px}pre{overflow:auto;background:#f1f5f9;padding:16px;line-height:1.45}pre code{padding:0}
figure{margin:24px 0;overflow:auto}figure svg{max-width:100%;height:auto}figcaption{color:#475569;font-size:0.9em}
@media print{body{max-width:none;font-size:10pt;padding:0}nav{break-after:page}section{break-before:page;border:0}pre{white-space:pre-wrap;overflow-wrap:anywhere}tr,figure{break-inside:avoid}a{color:inherit;text-decoration:none}}
@page{size:A4;margin:15mm}
</style></head><body>
<h1 id="handbook-top">Robot Motion C++ 0.13.0</h1>
<p>Integrationsgrundlage für eine selbst entwickelte Steuerung, Zyklusbereich 1–20 ms.
Diese Datei enthält Texte, UML- und Tutorial-Ablaufdiagramme sowie die Lehrkurve offline.
Weitere Mermaid-Ablaufquellen sind als Code erhalten;
GitHub rendert sie in der Markdown-Ausgabe. Druck/PDF ist über die Druckfunktion des Browsers möglich.</p>
<nav><ol>''' + ''.join(navigation) + '</ol></nav>' + ''.join(sections) + '</body></html>\n'
    check = AnchorCheck()
    check.feed(document)
    missing = check.targets - check.ids
    if missing:
        raise ValueError('Broken internal handbook anchors: ' + ', '.join(sorted(missing)))
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='fail if the checked-in HTML is out of date')
    options = parser.parse_args()
    document = build()
    if options.check:
        if not OUTPUT.is_file() or OUTPUT.read_text() != document:
            print('docs/handbook.html is out of date; run build_handbook.py', file=sys.stderr)
            return 1
    else:
        OUTPUT.write_text(document)
    print(f'Offline handbook: {len(CHAPTERS)} chapters, inline SVGs and verified local anchors')
    return 0


if __name__ == '__main__':
    sys.exit(main())
