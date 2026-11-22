# -*- coding: utf-8 -*-
import io
from docx import Document

path = r"D:\工创赛\报道文件\报到通知附件\报到通知附件\附件2-3智能救援赛项报到资格审查验收单.docx"
out = r"D:\工创赛\docx_dump.txt"

doc = Document(path)

lines = []
lines.append("===== PARAGRAPHS =====")
for i, p in enumerate(doc.paragraphs):
    txt = p.text.strip()
    if txt:
        lines.append(f"[{i}] ({p.style.name}) {txt}")

lines.append("")
lines.append("===== TABLES =====")
for ti, table in enumerate(doc.tables):
    lines.append(f"--- Table {ti} ({len(table.rows)} rows x {len(table.columns)} cols) ---")
    for ri, row in enumerate(table.rows):
        cells = [c.text.replace("\n", " | ").strip() for c in row.cells]
        lines.append(f"  R{ri}: {cells}")

with io.open(out, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

print("wrote", out)
