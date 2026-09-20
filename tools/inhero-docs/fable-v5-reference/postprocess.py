#!/usr/bin/env python3
"""
postprocess.py — macht die von mkdocs-static-i18n erzeugten seitenbezogenen
hreflang-Alternates absolut (Google-Anforderung) und ergaenzt x-default
(zeigt auf die deutsche Variante der jeweiligen Seite).
"""
import re
from pathlib import Path
from urllib.parse import urljoin

BASE = "https://docs.inhero.de"
SITE = Path(__file__).resolve().parent / "site"

ALT_RE = re.compile(
    r'<link rel="alternate" href="(?P<href>[^"]+)" hreflang="(?P<lang>[^"]+)"\s*/?>'
)


def page_url(html_file: Path) -> str:
    rel = html_file.relative_to(SITE).as_posix()
    if rel.endswith("index.html"):
        rel = rel[: -len("index.html")]
    return BASE + "/" + rel


def process(html_file: Path) -> bool:
    text = html_file.read_text(encoding="utf-8")
    base_url = page_url(html_file)
    changed = False
    de_abs = None

    def repl(m):
        nonlocal changed, de_abs
        absolute = urljoin(base_url, m.group("href"))
        if m.group("lang") == "de":
            de_abs = absolute
        changed = True
        return f'<link rel="alternate" href="{absolute}" hreflang="{m.group("lang")}">'

    text = ALT_RE.sub(repl, text)
    if de_abs and 'hreflang="x-default"' not in text:
        text = text.replace(
            f'<link rel="alternate" href="{de_abs}" hreflang="de">',
            f'<link rel="alternate" href="{de_abs}" hreflang="de">'
            f'<link rel="alternate" href="{de_abs}" hreflang="x-default">',
            1,
        )
    if changed:
        html_file.write_text(text, encoding="utf-8")
    return changed


def main():
    n = sum(process(f) for f in SITE.rglob("*.html"))
    print(f"hreflang absolutiert in {n} Dateien.")


if __name__ == "__main__":
    main()
