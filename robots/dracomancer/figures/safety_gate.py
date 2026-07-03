#!/usr/bin/env python3
"""Generate the dangerous-posture avoidance system overview SVG.

The figure is intentionally written with general labels so it can be reused in
presentations without exposing implementation-specific details.
"""

from pathlib import Path
from textwrap import dedent
from xml.sax.saxutils import escape


OUT = Path(__file__).with_suffix(".svg")


def text(x, y, body, class_name):
    return f'  <text class="{class_name}" x="{x}" y="{y}">{escape(body)}</text>'


def rect(x, y, width, height, class_name, radius=10):
    return (
        f'  <rect class="{class_name}" x="{x}" y="{y}" '
        f'width="{width}" height="{height}" rx="{radius}"/>'
    )


def path(d, class_name):
    return f'  <path class="{class_name}" d="{d}"/>'


def box(x, y, width, height, class_name, title, lines):
    margin_x = 28
    parts = [rect(x, y, width, height, class_name)]
    parts.append(text(x + margin_x, y + 36, title, "label"))
    for index, line in enumerate(lines):
        parts.append(text(x + margin_x, y + 66 + index * 24, line, "math"))
    return "\n".join(parts)


def main():
    svg = f"""\
<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="900" viewBox="0 0 1600 900">
  <defs>
    <marker id="arrow-blue" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#1f4e79"/>
    </marker>
    <marker id="arrow-green" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#2f6f4e"/>
    </marker>
    <marker id="arrow-red" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#9f2d2d"/>
    </marker>
    <marker id="arrow-gray" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#6b7280"/>
    </marker>
    <style>
      .bg {{ fill: #ffffff; }}
      .domain {{ fill: #f8fafc; stroke: #cbd5e1; stroke-width: 2; }}
      .domain-title {{ font: 700 24px Arial, Helvetica, sans-serif; fill: #1f2937; }}
      .box {{ fill: #ffffff; stroke: #334155; stroke-width: 2.2; }}
      .box-blue {{ fill: #f5fbff; stroke: #1f4e79; stroke-width: 2.5; }}
      .box-green {{ fill: #f4fbf6; stroke: #2f6f4e; stroke-width: 2.5; }}
      .box-amber {{ fill: #fff7e6; stroke: #9a6a00; stroke-width: 2.5; }}
      .box-red {{ fill: #fff0f0; stroke: #9f2d2d; stroke-width: 2.5; }}
      .label {{ font: 700 21px Arial, Helvetica, sans-serif; fill: #111827; }}
      .small {{ font: 400 17px Arial, Helvetica, sans-serif; fill: #374151; }}
      .math {{ font: 400 19px Consolas, Menlo, monospace; fill: #334155; }}
      .decision {{ fill: #fff7e6; stroke: #9a6a00; stroke-width: 2.7; }}
      .decision-text {{ font: 700 19px Consolas, Menlo, monospace; fill: #111827; text-anchor: middle; }}
      .arrow {{ fill: none; stroke: #1f4e79; stroke-width: 3.2; marker-end: url(#arrow-blue); }}
      .arrow-green {{ fill: none; stroke: #2f6f4e; stroke-width: 3.2; marker-end: url(#arrow-green); }}
      .arrow-red {{ fill: none; stroke: #9f2d2d; stroke-width: 3.2; marker-end: url(#arrow-red); }}
      .arrow-gray {{ fill: none; stroke: #6b7280; stroke-width: 2.7; marker-end: url(#arrow-gray); stroke-dasharray: 8 7; }}
    </style>
  </defs>

  <rect class="bg" x="0" y="0" width="1600" height="900"/>

  <rect class="domain" x="70" y="70" width="1460" height="205" rx="18"/>
{text(105, 122, "Posture request", "domain-title")}
{box(150, 165, 360, 74, "box-blue", "Exoskeleton joints", ["q_exo"])}
{box(620, 155, 400, 94, "box-blue", "Candidate posture", ["q^cand = f_map(q_exo)"])}
{path("M 510 202 L 620 202", "arrow")}

  <rect class="domain" x="70" y="340" width="1460" height="250" rx="18"/>
{text(105, 392, "Danger evaluation", "domain-title")}
{box(150, 455, 395, 104, "box-green", "Posture prediction", ["r_f(q^cand)", "r_tau(q^cand)"])}
{path("M 820 249 C 820 315, 348 315, 348 455", "arrow-gray")}
  <polygon class="decision" points="880,430 1090,505 880,580 670,505"/>
{text(880, 492, "r_f(q^cand) >= r_f^hard", "decision-text")}
{text(880, 520, "and r_tau(q^cand) >= r_tau^hard", "decision-text")}
{path("M 545 505 L 670 505", "arrow-green")}

  <rect class="domain" x="70" y="665" width="1460" height="185" rx="18"/>
{text(105, 731, "Avoidance action", "domain-title")}
{box(435, 715, 300, 74, "box-green", "Apply", ["q^tar = q^cand"])}
{box(900, 690, 430, 150, "box-red", "Avoid", ["q^tar = q^last", "release: r_f(q^cand) >= r_f^min", "and r_tau(q^cand) >= r_tau^min"])}
{path("M 880 580 C 835 653, 770 718, 735 752", "arrow-green")}
{text(790, 674, "safe", "small")}
{path("M 1090 505 C 1280 585, 1405 685, 1330 755", "arrow-red")}
{text(1210, 652, "danger", "small")}
</svg>
"""
    OUT.write_text(dedent(svg), encoding="utf-8")


if __name__ == "__main__":
    main()
