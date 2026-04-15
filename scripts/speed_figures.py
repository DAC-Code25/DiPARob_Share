#!/usr/bin/env python3
"""Generate speed-comparison figures."""

from __future__ import annotations

import argparse
import posixpath
from pathlib import Path
from typing import Dict, List
import zipfile
import xml.etree.ElementTree as ET

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, PngImagePlugin
from matplotlib.lines import Line2D


BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_STATISTICS_XLSX = BASE_DIR / "data" / "experiment_01" / "processed" / "error_statistics.xlsx"
DEFAULT_OUT_DIR = BASE_DIR / "data" / "experiment_01" / "figures"
MAIN_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
OFFICE_REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PACKAGE_REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"
NS = {"m": MAIN_NS, "r": OFFICE_REL_NS, "pr": PACKAGE_REL_NS}

SPEEDS = ["0.2", "0.3", "0.4"]
SPEED_COLORS = {"0.2": "#1F4E79", "0.3": "#2E8B57", "0.4": "#C65D3A"}
GRID_COLOR = "#D6DEE8"
PANEL_BG = "#FAFBFD"
TEXT_COLOR = "#243447"

mpl.rcParams.update(
    {
        "figure.dpi": 180,
        "savefig.dpi": 360,
        "font.family": "DejaVu Serif",
        "font.size": 10.5,
        "axes.labelsize": 11,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 9.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.unicode_minus": False,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.facecolor": "white",
    }
)

def style_axis(ax: plt.Axes, grid_axis: str | None = "y") -> None:
    ax.set_facecolor(PANEL_BG)
    if grid_axis is not None:
        ax.grid(True, axis=grid_axis, color=GRID_COLOR, linewidth=0.9, linestyle="--")
    else:
        ax.grid(False)
    ax.spines["left"].set_color("#7C8A97")
    ax.spines["bottom"].set_color("#7C8A97")
    ax.tick_params(colors="#5A6776")


def save_outputs(fig: plt.Figure, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / "Figure_generated.png"
    tif_path = out_dir / "Figure_generated.tif"
    fig.savefig(
        png_path,
        format="png",
        dpi=700,
        bbox_inches="tight",
        metadata={},
    )
    fig.savefig(
        tif_path,
        format="tiff",
        dpi=360,
        bbox_inches="tight",
        pil_kwargs={"compression": "tiff_lzw"},
    )
    png_image = Image.open(png_path)
    png_image.save(png_path, pnginfo=PngImagePlugin.PngInfo(), dpi=png_image.info.get("dpi", (700, 700)))
    tif_image = Image.open(tif_path)
    tif_image.save(tif_path, compression="tiff_lzw", dpi=tif_image.info.get("dpi", (360, 360)))


def column_name_to_index(cell_ref: str) -> int:
    letters = "".join(char for char in cell_ref if char.isalpha())
    index = 0
    for letter in letters.upper():
        index = index * 26 + ord(letter) - ord("A") + 1
    return index - 1


def load_shared_strings(archive: zipfile.ZipFile) -> List[str]:
    if "xl/sharedStrings.xml" not in archive.namelist():
        return []

    root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
    strings: List[str] = []
    for item in root.findall("m:si", NS):
        strings.append("".join((node.text or "") for node in item.findall(".//m:t", NS)))
    return strings


def workbook_sheet_targets(archive: zipfile.ZipFile) -> Dict[str, str]:
    workbook_root = ET.fromstring(archive.read("xl/workbook.xml"))
    rels_root = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
    rel_map = {
        rel.attrib["Id"]: rel.attrib["Target"]
        for rel in rels_root.findall("pr:Relationship", NS)
    }

    targets: Dict[str, str] = {}
    for sheet in workbook_root.findall("m:sheets/m:sheet", NS):
        name = sheet.attrib["name"]
        rel_id = sheet.attrib[f"{{{OFFICE_REL_NS}}}id"]
        target = rel_map[rel_id]
        if target.startswith("/"):
            targets[name] = target.lstrip("/")
        else:
            targets[name] = posixpath.normpath(posixpath.join("xl", target))
    return targets


def read_xlsx_sheet(path: Path, sheet_name: str) -> List[List[str]]:
    with zipfile.ZipFile(path) as archive:
        shared_strings = load_shared_strings(archive)
        sheet_targets = workbook_sheet_targets(archive)
        root = ET.fromstring(archive.read(sheet_targets[sheet_name]))

    rows: List[List[str]] = []
    for row in root.findall(".//m:sheetData/m:row", NS):
        values: List[str] = []
        for cell in row.findall("m:c", NS):
            column_index = column_name_to_index(cell.attrib.get("r", "A1"))
            while len(values) <= column_index:
                values.append("")

            cell_type = cell.attrib.get("t")
            value_node = cell.find("m:v", NS)
            inline_node = cell.find("m:is", NS)
            if cell_type == "s" and value_node is not None:
                value = shared_strings[int(value_node.text or "0")]
            elif cell_type == "inlineStr" and inline_node is not None:
                value = "".join(
                    (node.text or "") for node in inline_node.findall(".//m:t", NS)
                )
            elif value_node is not None:
                value = value_node.text or ""
            else:
                value = ""

            values[column_index] = value
        rows.append(values)
    return rows


def read_summary(path: Path) -> Dict[str, Dict[str, float]]:
    if path.suffix.lower() == ".xlsx":
        rows = read_xlsx_sheet(path, "Speed_Statistics")
        header = rows[0]
        indices = {name: index for index, name in enumerate(header)}
        out: Dict[str, Dict[str, float]] = {}
        for row in rows[1:]:
            speed = row[indices["Speed (m/s)"]]
            out[speed] = {
                "lat_mean_avg": float(row[indices["lateral error Mean (m)"]]),
                "lat_mean_rmse": float(row[indices["lateral error RMSE (m)"]]),
                "lat_mean_max": float(row[indices["lateral error Maximum (m)"]]),
                "yaw_mean_avg": float(row[indices["heading error Mean (deg)"]]),
                "yaw_mean_rmse": float(row[indices["heading error RMSE (deg)"]]),
                "yaw_mean_max": float(row[indices["heading error Maximum (deg)"]]),
            }
        return out

    out: Dict[str, Dict[str, float]] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            speed = row["speed"]
            out[speed] = {k: float(v) for k, v in row.items() if k not in {"speed", "bags"}}
    return out
def figure_01_summary_envelope(summary: Dict[str, Dict[str, float]], out_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11.4, 4.6))
    configs = [
        ("lat", "Lateral error (m)", 0.008, ".3f", axes[0]),
        ("yaw", "Heading error (°)", 0.32, ".2f", axes[1]),
    ]

    for prefix, ylabel, text_offset, fmt, ax in configs:
        style_axis(ax, None)
        xs = np.arange(len(SPEEDS))
        means = [summary[s][f"{prefix}_mean_avg"] for s in SPEEDS]
        rmses = [summary[s][f"{prefix}_mean_rmse"] for s in SPEEDS]
        maxs = [summary[s][f"{prefix}_mean_max"] for s in SPEEDS]

        ax.plot(xs, means, color="#5F6B7A", linewidth=1.6, linestyle="--", zorder=1)

        for x, speed, mean, rmse, max_value in zip(xs, SPEEDS, means, rmses, maxs):
            color = SPEED_COLORS[speed]
            ax.plot([x, x], [mean, max_value], color=color, linewidth=4.0, solid_capstyle="round", zorder=2)
            ax.scatter(x, mean, s=90, color=color, edgecolor="white", linewidth=1.1, zorder=4)
            ax.scatter(x, rmse, s=88, marker="s", facecolor="white", edgecolor=color, linewidth=2.1, zorder=5)
            ax.scatter(x, max_value, s=120, marker="D", facecolor=color, edgecolor="#21303A", linewidth=1.1, zorder=6)
            ax.text(x, max_value + text_offset, f"Max {format(max_value, fmt)}", ha="center", va="bottom", fontsize=9.2, color=color)
            x_shift = 0.07 if x < xs[-1] else -0.07
            ha = "left" if x < xs[-1] else "right"
            ax.text(
                x + x_shift,
                mean + text_offset * 0.55,
                f"Mean {format(mean, fmt)}",
                ha=ha,
                va="bottom",
                fontsize=8.9,
                color=TEXT_COLOR,
            )

        ax.set_xticks(xs)
        ax.set_xticklabels([f"{s} m/s" for s in SPEEDS])
        ax.set_xlabel("Speed")
        ax.set_ylabel(ylabel)

    legend_handles = [
        Line2D([0], [0], marker="o", linestyle="None", markerfacecolor="#4E6475", markeredgecolor="white", markersize=8.5, label="Mean"),
        Line2D([0], [0], marker="s", linestyle="None", markerfacecolor="white", markeredgecolor="#4E6475", markeredgewidth=2.0, markersize=8.0, label="RMSE"),
        Line2D([0], [0], marker="D", linestyle="None", markerfacecolor="#4E6475", markeredgecolor="#21303A", markersize=8.0, label="Max"),
    ]
    fig.legend(handles=legend_handles, loc="lower center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    save_outputs(fig, out_dir)
    plt.close(fig)
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--statistics-xlsx",
        type=Path,
        default=DEFAULT_STATISTICS_XLSX,
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
    )
    args = parser.parse_args()

    summary = read_summary(args.statistics_xlsx)
    figure_01_summary_envelope(summary, args.out_dir)


if __name__ == "__main__":
    main()
