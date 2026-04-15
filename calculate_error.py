# -*- coding: utf-8 -*-
"""Calculate lateral and heading errors from the inspection-vehicle log."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
import math
import posixpath
from pathlib import Path
from typing import Iterable, Sequence
import zipfile
import xml.etree.ElementTree as ET


BASE_DIR = Path(__file__).resolve().parent
INPUT_FILE_CANDIDATES = ("Test_Log.xlsx", "Test_Log（1.22）.xlsx")

WHEELBASE_M = 0.80
FULL_DATA_OUTPUT = "calculated_point_errors.xlsx"
STATISTICS_OUTPUT = "error_statistics.xlsx"

MAIN_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
OFFICE_REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PACKAGE_REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"

NS = {"m": MAIN_NS, "r": OFFICE_REL_NS, "pr": PACKAGE_REL_NS}


STYLE_DEFAULT = 0
STYLE_HEADER = 1
STYLE_TEXT = 2
STYLE_NUM_1 = 3
STYLE_NUM_2 = 4
STYLE_NUM_3 = 5
STYLE_NUM_4 = 6


@dataclass(frozen=True)
class Measurement:
    """One row of raw experimental measurements."""

    source_row: int
    sequence: str
    point: str
    speed_m_s: float
    direction: str
    front_deviation_m: float
    rear_deviation_m: float


@dataclass(frozen=True)
class ErrorRow:
    """One row of calculated row-level errors."""

    source_row: int
    sequence: str
    point: str
    speed_m_s: float
    direction: str
    front_deviation_m: float
    rear_deviation_m: float
    lateral_error_m: float
    heading_error_deg: float


@dataclass(frozen=True)
class SummaryRow:
    """Error statistics grouped by point or speed."""

    key: str | float
    lateral_mean_m: float
    lateral_rmse_m: float
    lateral_max_m: float
    heading_mean_deg: float
    heading_rmse_deg: float
    heading_max_deg: float

@dataclass(frozen=True)
class ExcelCell:
    """Minimal styled cell used by the local XLSX writer."""

    value: object
    style: int = STYLE_TEXT


def find_input_workbook(base_dir: Path = BASE_DIR) -> Path:
    """Return the raw workbook path, preferring the expected file names."""

    for file_name in INPUT_FILE_CANDIDATES:
        path = base_dir / file_name
        if path.exists():
            return path

    candidates = [
        path
        for path in sorted(base_dir.glob("Test_Log*.xlsx"))
        if not path.name.startswith(("calculated_", "error_"))
    ]
    if candidates:
        return candidates[0]

    expected = ", ".join(INPUT_FILE_CANDIDATES)
    raise FileNotFoundError(f"Raw workbook not found. Expected one of: {expected}")


def column_name_to_index(cell_ref: str) -> int:
    """Convert an Excel cell reference, such as ``C7``, to a zero-based index."""

    letters = "".join(char for char in cell_ref if char.isalpha())
    index = 0
    for letter in letters.upper():
        index = index * 26 + ord(letter) - ord("A") + 1
    return index - 1


def excel_column_name(index: int) -> str:
    """Convert a one-based column index to an Excel column name."""

    if index < 1:
        raise ValueError("Excel column index must be one-based.")

    name = ""
    while index:
        index, remainder = divmod(index - 1, 26)
        name = chr(ord("A") + remainder) + name
    return name


def load_shared_strings(archive: zipfile.ZipFile) -> list[str]:
    """Load the shared string table from an XLSX archive."""

    if "xl/sharedStrings.xml" not in archive.namelist():
        return []

    root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
    shared_strings: list[str] = []
    for item in root.findall("m:si", NS):
        text = "".join((node.text or "") for node in item.findall(".//m:t", NS))
        shared_strings.append(text)
    return shared_strings


def first_sheet_path(archive: zipfile.ZipFile) -> str:
    """Resolve the internal path of the first worksheet in a workbook."""

    workbook_root = ET.fromstring(archive.read("xl/workbook.xml"))
    first_sheet = workbook_root.find("m:sheets/m:sheet", NS)
    if first_sheet is None:
        raise ValueError("The workbook contains no worksheets.")

    relationship_id = first_sheet.attrib[f"{{{OFFICE_REL_NS}}}id"]
    rels_root = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
    relationships = {
        rel.attrib["Id"]: rel.attrib["Target"]
        for rel in rels_root.findall("pr:Relationship", NS)
    }
    target = relationships[relationship_id]
    if target.startswith("/"):
        return target.lstrip("/")
    return posixpath.normpath(posixpath.join("xl", target))


def read_first_sheet_rows(workbook_path: Path) -> list[list[str]]:
    """Read all rows from the first sheet of a simple XLSX workbook."""

    with zipfile.ZipFile(workbook_path) as archive:
        shared_strings = load_shared_strings(archive)
        sheet_path = first_sheet_path(archive)
        root = ET.fromstring(archive.read(sheet_path))

    rows: list[list[str]] = []
    for row in root.findall(".//m:sheetData/m:row", NS):
        values: list[str] = []
        for cell in row.findall("m:c", NS):
            cell_ref = cell.attrib.get("r", "A1")
            column_index = column_name_to_index(cell_ref)
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


def get_cell(row: Sequence[str], index: int) -> str:
    """Return a stripped row value, or an empty string for missing cells."""

    if index >= len(row):
        return ""
    return str(row[index]).strip()


def parse_float(value: str, row_number: int, field_name: str) -> float:
    """Parse a numeric value and report the workbook row when parsing fails."""

    normalized = value.replace(",", "").replace("−", "-").strip()
    try:
        return float(normalized)
    except ValueError as exc:
        raise ValueError(
            f"Unable to parse {field_name!r} as a number at workbook row {row_number}: "
            f"{value!r}"
        ) from exc


def read_measurements(workbook_path: Path) -> list[Measurement]:
    """Read raw measurements from the first worksheet."""

    rows = read_first_sheet_rows(workbook_path)
    if len(rows) < 2:
        raise ValueError(f"No data rows were found in {workbook_path.name}.")

    measurements: list[Measurement] = []
    current_sequence = ""
    for zero_based_index, row in enumerate(rows[1:], start=2):
        if not any(get_cell(row, index) for index in range(len(row))):
            continue

        sequence_cell = get_cell(row, 0)
        if sequence_cell:
            current_sequence = sequence_cell
        if not current_sequence:
            raise ValueError(f"Missing test sequence before workbook row {zero_based_index}.")

        point = get_cell(row, 1)
        direction = get_cell(row, 3)
        speed = parse_float(get_cell(row, 2), zero_based_index, "speed")
        front = parse_float(get_cell(row, 4), zero_based_index, "front deviation")
        rear = parse_float(get_cell(row, 5), zero_based_index, "rear deviation")

        measurements.append(
            Measurement(
                source_row=zero_based_index,
                sequence=current_sequence,
                point=point,
                speed_m_s=speed,
                direction=direction,
                front_deviation_m=front,
                rear_deviation_m=rear,
            )
        )

    return measurements


def calculate_errors(measurements: Iterable[Measurement]) -> list[ErrorRow]:
    """Calculate row-level lateral and heading errors."""

    calculated_rows: list[ErrorRow] = []
    for item in measurements:
        lateral_error = (item.front_deviation_m + item.rear_deviation_m) / 2.0
        heading_error = math.degrees(
            math.atan((item.front_deviation_m - item.rear_deviation_m) / WHEELBASE_M)
        )
        calculated_rows.append(
            ErrorRow(
                source_row=item.source_row,
                sequence=item.sequence,
                point=item.point,
                speed_m_s=item.speed_m_s,
                direction=item.direction,
                front_deviation_m=item.front_deviation_m,
                rear_deviation_m=item.rear_deviation_m,
                lateral_error_m=lateral_error,
                heading_error_deg=heading_error,
            )
        )
    return calculated_rows


def rmse(values: Sequence[float]) -> float:
    """Return the root mean square of a non-empty numeric sequence."""

    if not values:
        raise ValueError("RMSE requires at least one value.")
    return math.sqrt(sum(value * value for value in values) / len(values))


def summarize_errors(rows: Sequence[ErrorRow], group_by: str) -> list[SummaryRow]:
    """Summarize absolute error levels by ``point`` or ``speed``."""

    if group_by not in {"point", "speed"}:
        raise ValueError("group_by must be either 'point' or 'speed'.")

    groups: dict[str | float, list[ErrorRow]] = {}
    for row in rows:
        key: str | float = row.point if group_by == "point" else row.speed_m_s
        groups.setdefault(key, []).append(row)

    def sort_key(key: str | float) -> tuple[int, float | str]:
        if isinstance(key, float):
            return (0, key)
        if key.startswith("P") and key[1:].isdigit():
            return (0, int(key[1:]))
        return (1, key)

    summary: list[SummaryRow] = []
    for key in sorted(groups, key=sort_key):
        group_rows = groups[key]
        lateral_values = [abs(row.lateral_error_m) for row in group_rows]
        heading_values = [abs(row.heading_error_deg) for row in group_rows]
        summary.append(
            SummaryRow(
                key=key,
                lateral_mean_m=sum(lateral_values) / len(lateral_values),
                lateral_rmse_m=rmse(lateral_values),
                lateral_max_m=max(lateral_values),
                heading_mean_deg=sum(heading_values) / len(heading_values),
                heading_rmse_deg=rmse(heading_values),
                heading_max_deg=max(heading_values),
            )
        )
    return summary


def calculated_data_sheet(rows: Sequence[ErrorRow]) -> tuple[str, list[str], list[list[ExcelCell]]]:
    """Build the row-level worksheet payload."""

    headers = [
        "测试序列",
        "点位",
        "速度(m/s)",
        "方向",
        "前端偏差(m)",
        "后端偏差(m)",
        "横向误差(m)",
        "航向误差(deg)",
    ]
    data_rows = [
        [
            ExcelCell(row.sequence),
            ExcelCell(row.point),
            ExcelCell(row.speed_m_s, STYLE_NUM_1),
            ExcelCell(row.direction),
            ExcelCell(row.front_deviation_m, STYLE_NUM_3),
            ExcelCell(row.rear_deviation_m, STYLE_NUM_3),
            ExcelCell(row.lateral_error_m, STYLE_NUM_4),
            ExcelCell(row.heading_error_deg, STYLE_NUM_2),
        ]
        for row in rows
    ]
    return ("Calculated_Errors", headers, data_rows)


def summary_sheet(
    name: str, first_header: str, rows: Sequence[SummaryRow], key_style: int
) -> tuple[str, list[str], list[list[ExcelCell]]]:
    """Build a statistics worksheet payload."""

    headers = [
        first_header,
        "lateral error Mean (m)",
        "lateral error RMSE (m)",
        "lateral error Maximum (m)",
        "heading error Mean (deg)",
        "heading error RMSE (deg)",
        "heading error Maximum (deg)",
    ]
    data_rows = [
        [
            ExcelCell(row.key, key_style),
            ExcelCell(row.lateral_mean_m, STYLE_NUM_4),
            ExcelCell(row.lateral_rmse_m, STYLE_NUM_4),
            ExcelCell(row.lateral_max_m, STYLE_NUM_4),
            ExcelCell(row.heading_mean_deg, STYLE_NUM_3),
            ExcelCell(row.heading_rmse_deg, STYLE_NUM_3),
            ExcelCell(row.heading_max_deg, STYLE_NUM_3),
        ]
        for row in rows
    ]
    return (name, headers, data_rows)


def xml_text(value: object) -> str:
    """Escape cell text for XML."""

    return escape(str(value), quote=False)


def xml_attr(value: object) -> str:
    """Escape an XML attribute value."""

    return escape(str(value), quote=True)


def numeric_text(value: object) -> str:
    """Format a numeric value for XLSX XML."""

    if isinstance(value, int):
        return str(value)
    return f"{float(value):.15g}"


def cell_xml(row_index: int, column_index: int, cell: ExcelCell) -> str:
    """Serialize one cell."""

    ref = f"{excel_column_name(column_index)}{row_index}"
    style = cell.style
    value = cell.value

    if value is None or value == "":
        return f'<c r="{ref}" s="{style}"/>'

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f'<c r="{ref}" s="{style}"><v>{numeric_text(value)}</v></c>'

    text = xml_text(value)
    space = ' xml:space="preserve"' if text != text.strip() else ""
    return f'<c r="{ref}" t="inlineStr" s="{style}"><is><t{space}>{text}</t></is></c>'


def worksheet_xml(headers: Sequence[str], rows: Sequence[Sequence[ExcelCell]]) -> str:
    """Serialize a worksheet containing a header row and data rows."""

    all_rows = [[ExcelCell(header, STYLE_HEADER) for header in headers], *rows]
    max_column = max((len(row) for row in all_rows), default=1)
    dimension = f"A1:{excel_column_name(max_column)}{len(all_rows)}"

    column_widths = []
    for column_index in range(1, max_column + 1):
        width = 18 if column_index > 1 else 20
        column_widths.append(
            f'<col min="{column_index}" max="{column_index}" width="{width}" customWidth="1"/>'
        )

    row_xml = []
    for row_index, row in enumerate(all_rows, start=1):
        cells = "".join(
            cell_xml(row_index, column_index, cell)
            for column_index, cell in enumerate(row, start=1)
        )
        row_xml.append(f'<row r="{row_index}">{cells}</row>')

    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<worksheet xmlns="{MAIN_NS}" xmlns:r="{OFFICE_REL_NS}">'
        f'<dimension ref="{dimension}"/>'
        '<sheetViews><sheetView workbookViewId="0">'
        '<pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/>'
        '<selection pane="bottomLeft"/>'
        '</sheetView></sheetViews>'
        f'<cols>{"".join(column_widths)}</cols>'
        f'<sheetData>{"".join(row_xml)}</sheetData>'
        '<pageMargins left="0.7" right="0.7" top="0.75" bottom="0.75" '
        'header="0.3" footer="0.3"/>'
        '</worksheet>'
    )


def styles_xml() -> str:
    """Return the workbook style sheet."""

    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<styleSheet xmlns="{MAIN_NS}">'
        '<numFmts count="4">'
        '<numFmt numFmtId="164" formatCode="0.0"/>'
        '<numFmt numFmtId="165" formatCode="0.00"/>'
        '<numFmt numFmtId="166" formatCode="0.000"/>'
        '<numFmt numFmtId="167" formatCode="0.0000"/>'
        '</numFmts>'
        '<fonts count="2">'
        '<font><sz val="11"/><name val="Times New Roman"/><family val="1"/></font>'
        '<font><b/><sz val="11"/><name val="Times New Roman"/><family val="1"/></font>'
        '</fonts>'
        '<fills count="2">'
        '<fill><patternFill patternType="none"/></fill>'
        '<fill><patternFill patternType="gray125"/></fill>'
        '</fills>'
        '<borders count="2">'
        '<border><left/><right/><top/><bottom/><diagonal/></border>'
        '<border>'
        '<left style="thin"><color auto="1"/></left>'
        '<right style="thin"><color auto="1"/></right>'
        '<top style="thin"><color auto="1"/></top>'
        '<bottom style="thin"><color auto="1"/></bottom>'
        '<diagonal/>'
        '</border>'
        '</borders>'
        '<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" '
        'borderId="0"/></cellStyleXfs>'
        '<cellXfs count="7">'
        '<xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>'
        '<xf numFmtId="0" fontId="1" fillId="0" borderId="1" xfId="0" '
        'applyFont="1" applyBorder="1" applyAlignment="1">'
        '<alignment horizontal="center" vertical="center" wrapText="1"/></xf>'
        '<xf numFmtId="0" fontId="0" fillId="0" borderId="1" xfId="0" '
        'applyBorder="1" applyAlignment="1">'
        '<alignment horizontal="center" vertical="center"/></xf>'
        '<xf numFmtId="164" fontId="0" fillId="0" borderId="1" xfId="0" '
        'applyNumberFormat="1" applyBorder="1" applyAlignment="1">'
        '<alignment horizontal="center" vertical="center"/></xf>'
        '<xf numFmtId="165" fontId="0" fillId="0" borderId="1" xfId="0" '
        'applyNumberFormat="1" applyBorder="1" applyAlignment="1">'
        '<alignment horizontal="center" vertical="center"/></xf>'
        '<xf numFmtId="166" fontId="0" fillId="0" borderId="1" xfId="0" '
        'applyNumberFormat="1" applyBorder="1" applyAlignment="1">'
        '<alignment horizontal="center" vertical="center"/></xf>'
        '<xf numFmtId="167" fontId="0" fillId="0" borderId="1" xfId="0" '
        'applyNumberFormat="1" applyBorder="1" applyAlignment="1">'
        '<alignment horizontal="center" vertical="center"/></xf>'
        '</cellXfs>'
        '<cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/>'
        '</cellStyles>'
        '<dxfs count="0"/>'
        '<tableStyles count="0" defaultTableStyle="TableStyleMedium2" '
        'defaultPivotStyle="PivotStyleLight16"/>'
        '</styleSheet>'
    )


def content_types_xml(sheet_count: int) -> str:
    """Return [Content_Types].xml."""

    worksheet_overrides = "".join(
        '<Override PartName="/xl/worksheets/sheet{index}.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.'
        'spreadsheetml.worksheet+xml"/>'.format(index=index)
        for index in range(1, sheet_count + 1)
    )
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/xl/workbook.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
        f"{worksheet_overrides}"
        '<Override PartName="/xl/styles.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
        '<Override PartName="/docProps/core.xml" '
        'ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
        '<Override PartName="/docProps/app.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
        '</Types>'
    )


def package_rels_xml() -> str:
    """Return package relationship XML."""

    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<Relationships xmlns="{PACKAGE_REL_NS}">'
        '<Relationship Id="rId1" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" '
        'Target="xl/workbook.xml"/>'
        '<Relationship Id="rId2" '
        'Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" '
        'Target="docProps/core.xml"/>'
        '<Relationship Id="rId3" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" '
        'Target="docProps/app.xml"/>'
        '</Relationships>'
    )


def workbook_xml(sheet_names: Sequence[str]) -> str:
    """Return workbook XML."""

    sheets = "".join(
        f'<sheet name="{xml_attr(name[:31])}" sheetId="{index}" r:id="rId{index}"/>'
        for index, name in enumerate(sheet_names, start=1)
    )
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<workbook xmlns="{MAIN_NS}" xmlns:r="{OFFICE_REL_NS}">'
        '<workbookPr date1904="false"/>'
        f'<sheets>{sheets}</sheets>'
        '</workbook>'
    )


def workbook_rels_xml(sheet_count: int) -> str:
    """Return workbook relationships XML."""

    sheet_relationships = "".join(
        '<Relationship Id="rId{index}" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" '
        'Target="worksheets/sheet{index}.xml"/>'.format(index=index)
        for index in range(1, sheet_count + 1)
    )
    styles_id = sheet_count + 1
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<Relationships xmlns="{PACKAGE_REL_NS}">'
        f"{sheet_relationships}"
        f'<Relationship Id="rId{styles_id}" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" '
        'Target="styles.xml"/>'
        '</Relationships>'
    )


def core_props_xml() -> str:
    """Return document core properties."""

    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<cp:coreProperties '
        'xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" '
        'xmlns:dcterms="http://purl.org/dc/terms/" '
        'xmlns:dcmitype="http://purl.org/dc/dcmitype/" '
        'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"/>'
    )


def app_props_xml(sheet_names: Sequence[str]) -> str:
    """Return document extended properties."""

    sheet_entries = "".join(f"<vt:lpstr>{xml_text(name[:31])}</vt:lpstr>" for name in sheet_names)
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
        'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
        '<Application/>'
        '<DocSecurity>0</DocSecurity>'
        '<ScaleCrop>false</ScaleCrop>'
        '<HeadingPairs><vt:vector size="2" baseType="variant">'
        '<vt:variant><vt:lpstr>Worksheets</vt:lpstr></vt:variant>'
        f'<vt:variant><vt:i4>{len(sheet_names)}</vt:i4></vt:variant>'
        '</vt:vector></HeadingPairs>'
        f'<TitlesOfParts><vt:vector size="{len(sheet_names)}" baseType="lpstr">'
        f"{sheet_entries}"
        '</vt:vector></TitlesOfParts>'
        '</Properties>'
    )


def write_xlsx(
    output_path: Path,
    sheets: Sequence[tuple[str, list[str], list[list[ExcelCell]]]],
) -> None:
    """Write a simple multi-sheet XLSX workbook without third-party packages."""

    if not sheets:
        raise ValueError("At least one worksheet is required.")

    sheet_names = [sheet[0] for sheet in sheets]
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types_xml(len(sheets)))
        archive.writestr("_rels/.rels", package_rels_xml())
        archive.writestr("docProps/core.xml", core_props_xml())
        archive.writestr("docProps/app.xml", app_props_xml(sheet_names))
        archive.writestr("xl/workbook.xml", workbook_xml(sheet_names))
        archive.writestr("xl/_rels/workbook.xml.rels", workbook_rels_xml(len(sheets)))
        archive.writestr("xl/styles.xml", styles_xml())
        for index, (_, headers, rows) in enumerate(sheets, start=1):
            archive.writestr(
                f"xl/worksheets/sheet{index}.xml",
                worksheet_xml(headers, rows),
            )


def build_outputs(input_workbook: Path) -> tuple[list[ErrorRow], list[SummaryRow], list[SummaryRow]]:
    """Read the raw workbook and build all calculated outputs."""

    measurements = read_measurements(input_workbook)
    calculated_rows = calculate_errors(measurements)
    point_summary = summarize_errors(calculated_rows, group_by="point")
    speed_summary = summarize_errors(calculated_rows, group_by="speed")
    return calculated_rows, point_summary, speed_summary


def main() -> None:
    """Entry point used from the command line."""

    input_workbook = find_input_workbook()
    calculated_rows, point_summary, speed_summary = build_outputs(input_workbook)

    full_data_path = BASE_DIR / FULL_DATA_OUTPUT
    statistics_path = BASE_DIR / STATISTICS_OUTPUT

    write_xlsx(full_data_path, [calculated_data_sheet(calculated_rows)])
    write_xlsx(
        statistics_path,
        [
            summary_sheet("Point_Statistics", "Point", point_summary, STYLE_TEXT),
            summary_sheet("Speed_Statistics", "Speed (m/s)", speed_summary, STYLE_NUM_1),
        ],
    )


if __name__ == "__main__":
    main()
