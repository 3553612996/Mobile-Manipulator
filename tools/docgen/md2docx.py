#!/usr/bin/env python3
"""Convert the project's Markdown docs to .docx.

The Markdown files stay the source of truth: they diff, they link to each
other, and they render on GitHub.  This script only produces a Word copy for
reading, printing or sending to someone who expects a .docx.

Usage:
    python tools/docgen/md2docx.py          # docs/**/*.md -> docs/word/**/*.docx
    python tools/docgen/md2docx.py a.md b.md  # explicit files

The docs/ tree is mirrored under docs/word/ (architecture/overview.md becomes
docs/word/architecture/overview.docx), so two README.md in different folders
cannot clobber each other.

Requires python-docx (pip install python-docx).
"""

from __future__ import annotations

import datetime
import re
import sys
from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.opc.constants import RELATIONSHIP_TYPE as RT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = Path(__file__).resolve().parents[2]   # tools/docgen/ -> repo root
DOCS = ROOT / "docs"
OUT = DOCS / "word"

BODY_ASCII = "Times New Roman"
BODY_CJK = "宋体"
HEAD_CJK = "黑体"
MONO = "Consolas"
CODE_SIZE = 8.0
CODE_FILL = "F2F2F2"
RULE_COLOR = "A6A6A6"
QUOTE_COLOR = "595959"


# ---------------------------------------------------------------- utilities

def set_run_font(run, ascii_name, cjk_name, size_pt,
                 bold=None, color=None, italic=None):
    run.font.name = ascii_name
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.get_or_add_rFonts()
    rfonts.set(qn("w:ascii"), ascii_name)
    rfonts.set(qn("w:hAnsi"), ascii_name)
    rfonts.set(qn("w:eastAsia"), cjk_name)
    run.font.size = Pt(size_pt)
    if bold is not None:
        run.font.bold = bold
    if italic is not None:
        run.font.italic = italic
    if color:
        run.font.color.rgb = RGBColor.from_string(color)
    # Keep Word's spell checker from underlining every Chinese word.
    lang = OxmlElement("w:lang")
    lang.set(qn("w:val"), "en-US")
    lang.set(qn("w:eastAsia"), "zh-CN")
    rpr.append(lang)


def shade_paragraph(paragraph, fill):
    ppr = paragraph._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    ppr.append(shd)


def paragraph_border(paragraph, edge="bottom", color=RULE_COLOR, size="6"):
    ppr = paragraph._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    el = OxmlElement(f"w:{edge}")
    el.set(qn("w:val"), "single")
    el.set(qn("w:sz"), size)
    el.set(qn("w:space"), "8")
    el.set(qn("w:color"), color)
    pbdr.append(el)
    ppr.append(pbdr)


def add_hyperlink(paragraph, url, text, size_pt):
    part = paragraph.part
    r_id = part.relate_to(url, RT.HYPERLINK, is_external=True)
    link = OxmlElement("w:hyperlink")
    link.set(qn("r:id"), r_id)
    run = OxmlElement("w:r")
    rpr = OxmlElement("w:rPr")
    rfonts = OxmlElement("w:rFonts")
    rfonts.set(qn("w:ascii"), MONO)
    rfonts.set(qn("w:hAnsi"), MONO)
    rfonts.set(qn("w:eastAsia"), BODY_CJK)
    rpr.append(rfonts)
    color = OxmlElement("w:color")
    color.set(qn("w:val"), "0563C1")
    rpr.append(color)
    underline = OxmlElement("w:u")
    underline.set(qn("w:val"), "single")
    rpr.append(underline)
    sz = OxmlElement("w:sz")
    sz.set(qn("w:val"), str(int(size_pt * 2)))
    rpr.append(sz)
    run.append(rpr)
    text_el = OxmlElement("w:t")
    text_el.set(qn("xml:space"), "preserve")
    text_el.text = text
    run.append(text_el)
    link.append(run)
    paragraph._p.append(link)


# ------------------------------------------------------------ inline spans

INLINE = re.compile(
    r"(`[^`]+`"                  # inline code
    r"|\*\*[^*]+\*\*"            # bold
    r"|\[[^\]]+\]\([^)]+\)"      # link
    r"|\*[^*]+\*)"               # italic
)


def add_inline(paragraph, text, size=10.5, bold=False):
    """Add text with **bold**, `code`, [links] and *italic* resolved."""
    pos = 0

    def plain(chunk, is_bold=False, italic=False):
        if not chunk:
            return
        run = paragraph.add_run(chunk)
        set_run_font(run, BODY_ASCII, BODY_CJK, size,
                     bold=bold or is_bold, italic=italic or None)

    for match in INLINE.finditer(text):
        plain(text[pos:match.start()])
        token = match.group(0)

        if token.startswith("`"):
            run = paragraph.add_run(token[1:-1])
            set_run_font(run, MONO, BODY_CJK, size - 0.5, bold=bold)
        elif token.startswith("**"):
            # Bold often wraps code spans ("**`shared/` 存在的理由**"), so
            # recurse instead of flattening the content to plain text.
            add_inline(paragraph, token[2:-2], size=size, bold=True)
        elif token.startswith("["):
            label, url = re.match(r"\[([^\]]+)\]\(([^)]+)\)", token).groups()
            if url.startswith(("http://", "https://")):
                add_hyperlink(paragraph, url, label, size)
            else:
                plain(label)
        else:
            plain(token[1:-1], italic=True)

        pos = match.end()

    plain(text[pos:])


def join_lines(lines):
    """Markdown soft breaks render as a space; keep CJK text tight."""
    out = ""
    for raw in lines:
        chunk = raw.strip()
        if not chunk:
            continue
        if out and out[-1].isascii() and out[-1].isalnum() \
                and chunk[0].isascii() and chunk[0].isalnum():
            out += " "
        out += chunk
    return out


# ------------------------------------------------------------------ blocks

def add_heading(doc, level, text):
    sizes = {1: 20, 2: 15, 3: 12.5, 4: 11, 5: 10.5, 6: 10.5}
    paragraph = doc.add_paragraph()
    paragraph.style = doc.styles[f"Heading {min(level, 4)}"]
    fmt = paragraph.paragraph_format
    fmt.space_before = Pt(0 if level == 1 else 14 if level == 2 else 10)
    fmt.space_after = Pt(10 if level == 1 else 6 if level == 2 else 4)
    fmt.keep_with_next = True
    # Headings may carry code spans, e.g. "5.1 任务状态机（`robot_task`）".
    size = sizes.get(level, 10.5)
    for index, chunk in enumerate(re.split(r"`([^`]+)`", text)):
        if not chunk:
            continue
        run = paragraph.add_run(chunk)
        if index % 2 == 1:
            set_run_font(run, MONO, HEAD_CJK, size - 1.5,
                         bold=True, color="000000")
        else:
            set_run_font(run, HEAD_CJK, HEAD_CJK, size,
                         bold=True, color="000000")
    if level == 1:
        paragraph_border(paragraph, "bottom", "BFBFBF", "8")
    return paragraph


def add_code_block(doc, lines):
    paragraph = doc.add_paragraph()
    fmt = paragraph.paragraph_format
    fmt.left_indent = Cm(0.2)
    fmt.right_indent = Cm(0.1)
    fmt.space_before = Pt(4)
    fmt.space_after = Pt(8)
    fmt.line_spacing = 1.0
    shade_paragraph(paragraph, CODE_FILL)
    for index, line in enumerate(lines):
        run = paragraph.add_run(line)
        set_run_font(run, MONO, BODY_CJK, CODE_SIZE)
        if index < len(lines) - 1:
            run.add_break()


def add_quote(doc, lines):
    paragraph = doc.add_paragraph()
    fmt = paragraph.paragraph_format
    fmt.left_indent = Cm(0.6)
    fmt.space_before = Pt(4)
    fmt.space_after = Pt(8)
    fmt.line_spacing = 1.35
    paragraph_border(paragraph, "left", "BFBFBF", "12")

    for index, raw in enumerate(lines):
        text = raw.strip()
        item = LIST_ITEM.match(raw)
        if item:
            marker = item.group(2)
            if marker[0].isdigit():
                text = f"{marker.rstrip('.)')}. {item.group(3)}"
            else:
                text = f"• {item.group(3)}"
        if index:
            paragraph.add_run().add_break()
        if text:
            add_inline(paragraph, text, size=10.5)

    for run in paragraph.runs:
        run.font.color.rgb = RGBColor.from_string(QUOTE_COLOR)


def add_rule(doc):
    paragraph = doc.add_paragraph()
    paragraph.paragraph_format.space_before = Pt(2)
    paragraph.paragraph_format.space_after = Pt(2)
    paragraph_border(paragraph, "bottom", RULE_COLOR, "6")


LIST_ITEM = re.compile(r"^(\s*)([-*+]|\d+[.)])\s+(.*)$")
CHECKBOX = re.compile(r"^\[([ xX])\]\s+(.*)$")


def add_list_item(doc, indent, marker, content):
    level = min(indent // 2, 2)
    checkbox = CHECKBOX.match(content)
    if checkbox:
        marker = "☑" if checkbox.group(1).lower() == "x" else "☐"
        content = checkbox.group(2)
        is_ordered = False
    else:
        is_ordered = marker[0].isdigit()

    paragraph = doc.add_paragraph()
    fmt = paragraph.paragraph_format
    fmt.left_indent = Cm(0.7 + 0.65 * level)
    fmt.first_line_indent = Cm(-0.5)
    fmt.space_before = Pt(0)
    fmt.space_after = Pt(3)
    fmt.line_spacing = 1.35

    # Markdown's ordered marker already carries its own "." or ")"; unordered
    # markers are replaced by real bullets so the result does not look like a
    # leftover hyphen.
    if is_ordered:
        prefix = f"{marker.rstrip('.)')}. "
    elif marker in ("☐", "☑"):
        prefix = f"{marker} "
    else:
        prefix = "◦ " if level else "• "
    run = paragraph.add_run(prefix)
    set_run_font(run, BODY_ASCII, BODY_CJK, 10.5)
    add_inline(paragraph, content, size=10.5)


def split_row(line):
    """Split a pipe-table row, honouring \\| inside code spans."""
    cells, current, i = [], "", 0
    while i < len(line):
        char = line[i]
        if char == "\\" and i + 1 < len(line) and line[i + 1] == "|":
            current += "|"
            i += 2
            continue
        if char == "|":
            cells.append(current)
            current = ""
            i += 1
            continue
        current += char
        i += 1
    cells.append(current)
    if cells and not cells[0].strip():
        cells = cells[1:]
    if cells and not cells[-1].strip():
        cells = cells[:-1]
    return [cell.strip() for cell in cells]


SEPARATOR_CELL = re.compile(r"^:?-{2,}:?$")


def is_table_start(lines, index):
    if index + 1 >= len(lines):
        return False
    if not lines[index].lstrip().startswith("|"):
        return False
    cells = split_row(lines[index + 1].strip())
    return bool(cells) and all(SEPARATOR_CELL.match(cell) for cell in cells)


def add_table(doc, rows):
    separator = split_row(rows[1].strip())
    aligns = []
    for cell in separator:
        if cell.startswith(":") and cell.endswith(":"):
            aligns.append(WD_ALIGN_PARAGRAPH.CENTER)
        elif cell.endswith(":"):
            aligns.append(WD_ALIGN_PARAGRAPH.RIGHT)
        else:
            aligns.append(WD_ALIGN_PARAGRAPH.LEFT)

    body = [split_row(row.strip()) for row in rows[2:]]
    columns = max([len(separator)] + [len(row) for row in body])

    table = doc.add_table(rows=len(body) + 1, cols=columns)
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = True

    header = table.rows[0]
    header_props = header._tr.get_or_add_trPr()
    repeat = OxmlElement("w:tblHeader")
    repeat.set(qn("w:val"), "true")
    header_props.append(repeat)

    for row_index, cells in enumerate([split_row(rows[0].strip())] + body):
        row = table.rows[row_index]
        # A row split across a page break loses its first half - keep it whole.
        row._tr.get_or_add_trPr().append(OxmlElement("w:cantSplit"))
        for column in range(columns):
            cell = row.cells[column]
            paragraph = cell.paragraphs[0]
            text = cells[column] if column < len(cells) else ""
            if row_index == 0:
                tcpr = cell._tc.get_or_add_tcPr()
                shd = OxmlElement("w:shd")
                shd.set(qn("w:val"), "clear")
                shd.set(qn("w:color"), "auto")
                shd.set(qn("w:fill"), "EDEDED")
                tcpr.append(shd)
            paragraph.alignment = aligns[column] if column < len(aligns) \
                else WD_ALIGN_PARAGRAPH.LEFT
            paragraph.paragraph_format.space_before = Pt(1)
            paragraph.paragraph_format.space_after = Pt(1)
            add_inline(paragraph, text, size=9, bold=(row_index == 0))

    doc.add_paragraph().paragraph_format.space_after = Pt(2)


# ------------------------------------------------------------------- setup

def setup_document(doc, title):
    section = doc.sections[0]
    section.page_width, section.page_height = Cm(21), Cm(29.7)
    section.left_margin = section.right_margin = Cm(2.2)
    section.top_margin, section.bottom_margin = Cm(2.5), Cm(2.2)

    normal = doc.styles["Normal"]
    normal.font.name = BODY_ASCII
    normal.font.size = Pt(10.5)
    rpr = normal.element.get_or_add_rPr()
    rfonts = rpr.get_or_add_rFonts()
    rfonts.set(qn("w:ascii"), BODY_ASCII)
    rfonts.set(qn("w:hAnsi"), BODY_ASCII)
    rfonts.set(qn("w:eastAsia"), BODY_CJK)
    normal.paragraph_format.line_spacing = 1.4
    normal.paragraph_format.space_after = Pt(6)

    core = doc.core_properties
    core.title = title
    core.author = "wuying"
    core.subject = "Mobile-Manipulator 项目文档"

    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = footer.add_run(
        f"{title}  ·  Mobile-Manipulator  ·  "
        f"{datetime.date.today().isoformat()}  ·  第 ")
    set_run_font(run, BODY_ASCII, BODY_CJK, 8.5, color="808080")
    field = footer.add_run()
    set_run_font(field, BODY_ASCII, BODY_CJK, 8.5, color="808080")
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = "PAGE"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    field._r.append(begin)
    field._r.append(instr)
    field._r.append(end)
    tail = footer.add_run(" 页")
    set_run_font(tail, BODY_ASCII, BODY_CJK, 8.5, color="808080")


# ---------------------------------------------------------------- converter

def convert(md_path, out_path):
    lines = md_path.read_text(encoding="utf-8").splitlines()

    title = next((re.match(r"^#\s+(.*)$", line).group(1)
                  for line in lines if re.match(r"^#\s+", line)),
                 md_path.stem)

    doc = Document()
    setup_document(doc, title)

    index = 0
    while index < len(lines):
        line = lines[index]

        fence = line.strip()
        if fence.startswith("```"):
            index += 1
            code = []
            while index < len(lines) and not lines[index].strip().startswith("```"):
                code.append(lines[index])
                index += 1
            index += 1
            add_code_block(doc, code or [""])
            continue

        if is_table_start(lines, index):
            table_lines = []
            while index < len(lines) and lines[index].strip().startswith("|"):
                table_lines.append(lines[index])
                index += 1
            add_table(doc, table_lines)
            continue

        heading = re.match(r"^(#{1,6})\s+(.*?)\s*#*\s*$", line)
        if heading:
            add_heading(doc, len(heading.group(1)), heading.group(2))
            index += 1
            continue

        if re.match(r"^\s*([-*_])\s*\1\s*\1[\s\-*_]*$", line):
            add_rule(doc)
            index += 1
            continue

        if line.startswith(">"):
            quote = []
            while index < len(lines) and lines[index].startswith(">"):
                quote.append(re.sub(r"^>\s?", "", lines[index]))
                index += 1
            add_quote(doc, [part for part in quote])
            continue

        item = LIST_ITEM.match(line)
        if item:
            add_list_item(doc, len(item.group(1)), item.group(2), item.group(3))
            index += 1
            continue

        if not line.strip():
            index += 1
            continue

        paragraph_lines = []
        while index < len(lines):
            candidate = lines[index]
            if (not candidate.strip()
                    or candidate.strip().startswith("```")
                    or candidate.lstrip().startswith("|")
                    or re.match(r"^(#{1,6})\s+", candidate)
                    or candidate.startswith(">")
                    or LIST_ITEM.match(candidate)
                    or re.match(r"^\s*([-*_])\s*\1\s*\1[\s\-*_]*$", candidate)):
                break
            paragraph_lines.append(candidate.strip())
            index += 1

        text = join_lines(paragraph_lines)
        paragraph = doc.add_paragraph()
        add_inline(paragraph, text, size=10.5)

    doc.save(out_path)
    return title


def main(argv):
    if argv:
        # resolve() so a relative CLI path can still be placed with
        # relative_to(DOCS) below.
        sources = [Path(arg).resolve() for arg in argv]
    else:
        # rglob, not glob: the docs tree is split into architecture/,
        # conventions/ and nodes/<id>/, and the output mirrors it.
        sources = sorted(DOCS.rglob("*.md"))

    OUT.mkdir(parents=True, exist_ok=True)
    for source in sources:
        if "copy" in source.stem.lower():
            continue
        target = OUT / source.relative_to(DOCS).with_suffix(".docx")
        target.parent.mkdir(parents=True, exist_ok=True)
        title = convert(source, target)
        print(f"{str(source.relative_to(DOCS)):32s} -> "
              f"{target.relative_to(ROOT)}  ({title})")


if __name__ == "__main__":
    main(sys.argv[1:])
