#!/usr/bin/env python3
"""Generate launch/system.svg from Dracomancer's ROS launch XML."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[2]
LAUNCH = PACKAGE / "launch"
STYLES = {
    "launch": {"fillcolor": "#163e61", "color": "#163e61", "fontcolor": "#ffffff"},
    "python": {"fillcolor": "#e9f7f3", "color": "#438b7f", "fontcolor": "#123b38"},
    "program": {"fillcolor": "#fff5df", "color": "#b69249", "fontcolor": "#513d17"},
    "external": {"fillcolor": "#f0f3f7", "color": "#9aa8b8", "fontcolor": "#293c50"},
}


def attributes(values):
    return ", ".join(f"{name}={json.dumps(value, ensure_ascii=False)}"
                     for name, value in values.items())


def declarations(parent, guards=()):
    for kind in ("if", "unless"):
        if kind in parent.attrib:
            guards += (f"{kind}={parent.attrib[kind]}",)
    if parent.tag in ("include", "node"):
        yield parent, guards
    else:
        for child in parent:
            yield from declarations(child, guards)


def render_graph():
    launches = {path.relative_to(LAUNCH).as_posix(): path
                for path in sorted(LAUNCH.rglob("*.launch"))}
    scripts = defaultdict(list)
    script_root = PACKAGE / "scripts"
    # Catkin installs node executables by basename, so each must resolve uniquely.
    for path in sorted(script_root.rglob("*")):
        if path.is_file() and path.suffix in (".py", ".sh"):
            scripts[path.name].append(path.relative_to(PACKAGE).as_posix())

    nodes = {}
    edges = defaultdict(list)
    for name in launches:
        nodes[name] = ("launch", name)

    for name, path in launches.items():
        for element, guards in declarations(ET.parse(path).getroot()):
            if element.tag == "include":
                include = element.attrib["file"]
                match = re.fullmatch(r"\$\(find ([\w_]+)\)/launch/(.+)", include)
                if match and match[1] == "dracomancer":
                    target = match[2]
                    if target not in launches:
                        raise ValueError(f"{name}: included launch does not exist: {target}")
                else:
                    target = f"{match[1]}/{match[2]}" if match else include
                    nodes[target] = ("external", None)
            elif element.get("pkg") == "dracomancer":
                executable = element.attrib["type"]
                if Path(executable).suffix in (".py", ".sh"):
                    matches = scripts[Path(executable).name]
                    if len(matches) != 1:
                        raise ValueError(f"{name}: cannot resolve {executable}: {matches}")
                    target = matches[0]
                    kind = "python" if executable.endswith(".py") else "program"
                    nodes[target] = (kind, f"../{target}")
                else:
                    target = executable
                    nodes[target] = ("program", None)
            else:
                continue
            edges[name, target].append((element.attrib, guards))

    dot = [
        "digraph Dracomancer {",
        '  graph [rankdir=LR, bgcolor="#fbfcfe", pad=0.3, ranksep=1.1, nodesep=0.28, '
        'fontname="Noto Sans CJK JP", labelloc=t, labeljust=l];',
        '  node [shape=box, style="rounded,filled", fontname="Noto Sans CJK JP", '
        'fontsize=13, margin="0.15,0.10"];',
        '  edge [color="#718396", fontcolor="#40576c", fontname="Noto Sans CJK JP", '
        'fontsize=10, arrowsize=0.68];',
        '  label=<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4">',
        '  <TR><TD ALIGN="LEFT"><FONT POINT-SIZE="25" COLOR="#163e61">'
        '<B>Dracomancer Launch 構成</B></FONT></TD></TR>',
        '  <TR><TD ALIGN="LEFT">矢印は include または実行。破線は条件付き。</TD></TR>',
        '  <TR><TD ALIGN="LEFT"><FONT COLOR="#163e61">■ パッケージ内 launch</FONT>'
        '　<FONT COLOR="#277267">■ Python</FONT>　<FONT COLOR="#8c6a27">■ その他の実行ファイル</FONT>'
        '　<FONT COLOR="#5c7186">■ 外部 launch</FONT></TD></TR></TABLE>>;',
    ]
    for name, (kind, url) in nodes.items():
        style = {"label": name, **STYLES[kind]}
        if url:
            style["URL"] = url
        dot.append(f"  {json.dumps(name)} [{attributes(style)}];")
    for (source, target), calls in edges.items():
        modes = list(dict.fromkeys(item.get("args", "") for item, _ in calls))
        details = [
            " ".join((item.get("name", "include"), item.get("args", ""), *guards)).strip()
            for item, guards in calls
        ]
        style = {
            "style": "dashed" if all(guards for _, guards in calls) else "solid",
            "tooltip": " | ".join(details),
        }
        if any(modes):
            style["label"] = " / ".join(mode or "通常" for mode in modes)
        dot.append(f"  {json.dumps(source)} -> {json.dumps(target)} [{attributes(style)}];")
    return "\n".join([*dot, "}", ""])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=LAUNCH / "system.svg")
    args = parser.parse_args()
    try:
        svg = subprocess.run(["dot", "-Tsvg"], input=render_graph(), text=True,
                             capture_output=True, check=True).stdout
    except FileNotFoundError:
        parser.error("Graphviz dot is required")
    args.output.write_text(svg, encoding="utf-8")


if __name__ == "__main__":
    main()
