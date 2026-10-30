"""
build_report.py -- 보고서_본문.md 를 학교 양식(자산어보 최종 보고서) PDF 로 만듦

  python report_pdf/build_report.py

양식 PDF 에서 뽑은 글씨 크기와 위치를 그대로 씀.
  표지: 학기 줄 15pt, 주제 상자, 팀명과 이름 20pt, 날짜 22pt
  목차: 제목 32pt, 장 18pt, 절 14pt, 테두리 상자
  장 시작: 보라색 띠 그림 위에 파란 글씨 장 제목
본문 글씨 크기는 양식에 없어서 11pt 로 정함.
"""

import html
import re
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SOURCE = ROOT / "보고서_본문.md"
OUTPUT = HERE / "2026-2학기 자산어보 창의융합 프로젝트 최종 보고서(Tesla).pdf"
EDGE = Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")

TEAM_NAME = "Tesla"
# 표지에 학번 이름 순서로 한 줄씩 들어감
MEMBERS = ["학번 이름", "학번 이름", "학번 이름"]
DATE_TEXT = "2026년 11월"

ROMAN = {"I": "Ⅰ", "II": "Ⅱ", "III": "Ⅲ", "IV": "Ⅳ"}


def inline(text):
    """한 줄 안의 굵게, 기울임을 HTML 로 바꿈. 마크다운의 A\\* 같은 빗금도 지움."""
    text = html.escape(text.replace("\\*", "\x00"))
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
    text = re.sub(r"\*(.+?)\*", r"<i>\1</i>", text)
    return text.replace("\x00", "*")


def table_html(rows):
    cells = [[c.strip() for c in row.strip().strip("|").split("|")] for row in rows]
    head, body = cells[0], cells[2:]
    out = ["<table><thead><tr>"]
    out += [f"<th>{inline(c)}</th>" for c in head]
    out.append("</tr></thead><tbody>")
    for row in body:
        out.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in row) + "</tr>")
    out.append("</tbody></table>")
    return "".join(out)


def body_html(markdown_text):
    """본문을 장(# ) 단위로 HTML 로 바꿈. 맨 위의 보고서 제목과 메모(> )는 표지와 작업용이라 뺌."""
    lines = markdown_text.splitlines()
    out = []
    index = 0
    seen_chapter = False
    while index < len(lines):
        line = lines[index]
        stripped = line.strip()
        if stripped.startswith("# "):
            match = re.match(r"# (I{1,3}|IV)\. (.+)", stripped)
            if match:
                seen_chapter = True
                number, title = match.groups()
                out.append(f'<section class="chapter"><div class="banner">'
                           f'<span class="roman">{ROMAN[number]}.</span> {html.escape(title)}</div>')
            index += 1
            continue
        if not seen_chapter or not stripped or stripped == "---" or stripped.startswith(">"):
            index += 1
            continue
        if stripped.startswith("|"):
            # 표는 | 로 시작하는 줄이 끝날 때까지 한 덩어리로 모음
            block = []
            while index < len(lines) and lines[index].strip().startswith("|"):
                block.append(lines[index])
                index += 1
            out.append(table_html(block))
            continue
        if stripped.startswith("!["):
            caption, path = re.match(r"!\[(.*)\]\((.*)\)", stripped).groups()
            image = (ROOT / path).as_uri()
            out.append(f'<figure><img src="{image}"><figcaption>{inline(caption)}</figcaption></figure>')
        elif stripped.startswith("- "):
            items = []
            while index < len(lines) and lines[index].strip().startswith("- "):
                items.append(f"<li>{inline(lines[index].strip()[2:])}</li>")
                index += 1
            out.append("<ul>" + "".join(items) + "</ul>")
            continue
        elif stripped.startswith("### "):
            out.append(f"<h3>{inline(stripped[4:])}</h3>")
        elif stripped.startswith("## "):
            out.append(f"<h2>{inline(stripped[3:])}</h2>")
        elif re.match(r"\d+\. [A-Z]", stripped):
            out.append(f'<p class="reference">{inline(stripped)}</p>')
        else:
            out.append(f"<p>{inline(stripped)}</p>")
        index += 1
    return "\n".join(out)


def toc_html(markdown_text):
    """목차는 장과 절(## )까지만 씀. 양식처럼 두 쪽으로 나눠 III 장부터 둘째 쪽에 둠."""
    pages = [[], []]
    page = 0
    for line in markdown_text.splitlines():
        match = re.match(r"# (I{1,3}|IV)\. (.+)", line)
        if match:
            if match.group(1) == "III":
                page = 1
            pages[page].append(f'<div class="toc-chapter">{match.group(1)}. {html.escape(match.group(2))}</div>')
        elif line.startswith("## ") and page < 2:
            pages[page].append(f'<div class="toc-section">{inline(line[3:])}</div>')
    first = '<div class="toc-title">목 차</div>' + "".join(pages[0])
    return (f'<section class="toc"><div class="toc-box">{first}</div></section>'
            f'<section class="toc"><div class="toc-box">{"".join(pages[1])}</div></section>')


def cover_html(title):
    logo = (HERE / "template_p0.jpeg").as_uri()
    members = "".join(f'<div class="member">{html.escape(m)}</div>' for m in MEMBERS)
    return f"""
<section class="cover">
  <div class="term">2026학년도 2학기 자산어보 창의융합 프로젝트</div>
  <div class="bar" style="top:143pt"></div>
  <div class="title-box"><div>{html.escape(title)}</div></div>
  <div class="bar" style="top:296pt"></div>
  <img class="logo" src="{logo}">
  <div class="team">{html.escape(TEAM_NAME)}</div>
  <div class="members">{members}</div>
  <div class="date">{html.escape(DATE_TEXT)}</div>
</section>"""


STYLE = """
@font-face { font-family: "HanBatang"; src: url("file:///C:/Windows/Fonts/HANBatang.ttf"); }
@font-face { font-family: "HanBatang"; font-weight: bold; src: url("file:///C:/Windows/Fonts/HANBatangB.ttf"); }
/* 양식의 본문 쪽 여백: 왼쪽 30 mm, 오른쪽 15 mm, 위 20 mm */
@page { size: A4; margin: 20mm 15mm 20mm 30mm;
        @bottom-center { content: counter(page); font: 10pt "HanBatang"; } }
@page cover { margin: 0; @bottom-center { content: none; } }
@page toc { margin: 0; @bottom-center { content: none; } }
* { box-sizing: border-box; }
body { margin: 0; font-family: "HanBatang", "Batang", serif; font-size: 11pt; line-height: 1.7;
       color: #000; word-break: keep-all; }

.cover { page: cover; position: relative; width: 210mm; height: 297mm; break-after: page;
         font-family: "Malgun Gothic", sans-serif; }
.cover .term { position: absolute; left: 85pt; top: 55pt; font-size: 15pt; }
.cover .bar { position: absolute; left: 85pt; width: 468pt; height: 10pt; background: #000080; }
.cover .title-box { position: absolute; left: 85pt; top: 180pt; width: 468pt; height: 89pt;
                    background: #AEF6F6; display: flex; align-items: center; justify-content: center;
                    text-align: center; padding: 0 18pt; font-size: 21pt; line-height: 1.35; }
.cover .logo { position: absolute; left: 259pt; top: 416pt; width: 120pt; height: 120pt; }
.cover .team { position: absolute; left: 85pt; width: 468pt; top: 598pt; text-align: center; font-size: 20pt; }
.cover .members { position: absolute; left: 85pt; width: 468pt; top: 655pt; text-align: center;
                  font-size: 20pt; letter-spacing: 1pt; }
.cover .member { height: 40pt; }
.cover .date { position: absolute; left: 85pt; width: 468pt; top: 785pt; text-align: center; font-size: 22pt; }

.toc { page: toc; width: 210mm; height: 297mm; position: relative; break-after: page; }
.toc-box { position: absolute; left: 85pt; top: 57pt; width: 468pt; height: 710pt;
           border: 0.375pt solid #000; padding: 6pt 22pt; }
.toc-title { text-align: center; font-family: "Malgun Gothic", sans-serif; font-size: 32pt; margin: 4pt 0 40pt; }
.toc-chapter { font-size: 18pt; margin: 30pt 0 16pt; }
.toc-section { font-size: 14pt; margin: 0 0 24pt 7pt; }

.chapter { break-before: page; }
/* 띠 그림은 양식처럼 오른쪽 여백을 넘어 쪽 끝까지 감 */
.banner { margin: 0 -15mm 22pt 0; height: 75.6pt; background: url("BANNER") left center / 100% 100% no-repeat;
          color: #0000FF; font-family: "Malgun Gothic", sans-serif; font-weight: bold; font-size: 24pt;
          display: flex; align-items: center; justify-content: center; padding-right: 42pt; }
.banner .roman { font-size: 27pt; margin-right: 6pt; }

h2 { font-size: 14pt; font-weight: bold; margin: 20pt 0 8pt; break-after: avoid; }
h3 { font-size: 12pt; font-weight: bold; margin: 14pt 0 6pt; break-after: avoid; }
p { margin: 0 0 7pt; text-align: justify; }
ul { margin: 0 0 8pt; padding-left: 16pt; }
li { margin-bottom: 3pt; text-align: justify; }
table { width: 100%; border-collapse: collapse; margin: 6pt 0 12pt; font-size: 9.5pt; line-height: 1.45;
        break-inside: auto; }
th, td { border: 0.6pt solid #555; padding: 3pt 5pt; vertical-align: top; text-align: left; }
th { background: #E8EAF4; font-weight: bold; }
tr { break-inside: avoid; }
figure { margin: 8pt 0 14pt; text-align: center; break-inside: avoid; }
figure img { width: 88%; }
figcaption { font-size: 9.5pt; line-height: 1.45; margin-top: 4pt; }
.reference { padding-left: 18pt; text-indent: -18pt; text-align: left; }
"""


def main():
    markdown_text = SOURCE.read_text(encoding="utf-8")
    title = markdown_text.splitlines()[0].lstrip("# ").strip()
    style = STYLE.replace("BANNER", (HERE / "template_p4.jpeg").as_uri())
    page = (f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><style>{style}</style></head><body>'
            f"{cover_html(title)}{toc_html(markdown_text)}{body_html(markdown_text)}</body></html>")
    page_path = HERE / "report.html"
    page_path.write_text(page, encoding="utf-8")
    # 엣지를 화면 없이 띄워 인쇄함. 머리말 꼬리말(주소, 날짜)은 끔
    subprocess.run([str(EDGE), "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    "--allow-file-access-from-files", f"--print-to-pdf={OUTPUT}", page_path.as_uri()],
                   check=True, timeout=120)
    print(OUTPUT)


if __name__ == "__main__":
    main()
