"""Render the reviewed UML subset using local Graphviz; no network access.

The specification references real declarations. Relationships and displayed method
subsets are reviewed documentation, not inferred C++ ownership semantics.
"""
from pathlib import Path
import argparse
import html
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DIAGRAMS = ROOT / "docs/architecture/diagrams"


def validate_node(node):
    source = ROOT / node["header"]
    declaration = node["name"].split("<")[0]
    if not source.is_file() or not re.search(r"\b(?:class|struct)\s+" + declaration + r"\b", source.read_text()):
        raise ValueError(f"Missing documented declaration: {declaration} in {source}")
    for method in node.get("methods", []):
        name = method.split("(")[0].strip().split()[-1]
        if not re.search(r"\b" + re.escape(name) + r"\s*\(", source.read_text()):
            raise ValueError(f"Missing documented method: {declaration}::{name}")


def graph(spec):
    output = ['digraph uml {', 'graph [rankdir=LR, bgcolor="white", pad="0.25", nodesep="0.35", ranksep="0.8"];',
              'node [shape=plain, fontname="DejaVu Sans", fontsize=10];',
              'edge [fontname="DejaVu Sans", fontsize=9, color="#475569"];']
    identifiers = {node["id"] for node in spec["nodes"]}
    for node in spec["nodes"]:
        validate_node(node)
        title = html.escape(node["name"])
        cells = [f'<TR><TD BGCOLOR="#dbeafe"><B>{title}</B></TD></TR>']
        for row in node.get("fields", []) + node.get("methods", []):
            cells.append(f'<TR><TD ALIGN="LEFT">{html.escape(row)}</TD></TR>')
        output.append(f'{node["id"]} [label=<<TABLE BORDER="1" CELLBORDER="0" CELLSPACING="0" CELLPADDING="5">'+"".join(cells)+"</TABLE>>];")
    for edge in spec["edges"]:
        if edge[0] not in identifiers or edge[1] not in identifiers:
            raise ValueError(f"Unknown class in {edge}")
        kind, label = edge[2], edge[3]
        attributes = {'inherits': 'arrowhead=empty', 'owns': 'dir=both, arrowtail=diamond, arrowhead=none',
                      'borrows': 'style=dashed, arrowhead=vee', 'uses': 'style=dashed, arrowhead=vee'}[kind]
        output.append(f'{edge[0]} -> {edge[1]} [{attributes}, label={json.dumps(label, ensure_ascii=False)}];')
    output.append('}')
    return "\n".join(output) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", choices=["svg", "pdf"], default="svg")
    args = parser.parse_args()
    specs = json.loads((DIAGRAMS / "class_diagrams.json").read_text())
    for spec in specs:
        dot = DIAGRAMS / (spec["id"] + ".dot")
        dot.write_text(graph(spec))
        subprocess.run(["dot", "-T" + args.format, str(dot), "-o", str(dot.with_suffix("." + args.format))], check=True)
    # The module diagram is measured from the same dependency policy used in CI.
    from check_layers import inspect
    policy = json.loads((ROOT / 'docs/architecture/layers.json').read_text())
    report = inspect(ROOT, policy)
    if report['errors']:
        raise ValueError('\n'.join(report['errors']))
    lines = ['digraph modules {', 'rankdir=BT;',
             'node [shape=box, style="rounded,filled", fillcolor="#dbeafe", fontname="DejaVu Sans"];',
             'edge [color="#64748b"];']
    for name, dependencies in report['layers'].items():
        lines.append(json.dumps(name) + ';')
        for dependency in dependencies:
            lines.append(json.dumps(name) + ' -> ' + json.dumps(dependency) + ';')
    lines.append('}')
    module_dot = DIAGRAMS / 'modules.dot'
    module_dot.write_text('\n'.join(lines) + '\n')
    subprocess.run(['dot', '-T' + args.format, str(module_dot), '-o', str(module_dot.with_suffix('.' + args.format))], check=True)
    print(f"Validated declarations and rendered {len(specs)} UML diagrams ({args.format})")


if __name__ == "__main__":
    main()
