#!/usr/bin/env python3
"""
check_links.py — prueft nach dem Build, ob alle Bildverweise und internen
Links in site/ tatsaechlich aufloesen, und ob jeder Anker (#fragment) auf
eine existierende Ueberschrift zeigt. Bricht mit Exit-Code 1 ab, wenn nicht.

Hintergrund Datei-Ebene: Die Quelldateien mischen Markdown- und HTML-Syntax
fuer Bilder, und die DE-Fassung liegt im Quell-Repo eine Ebene tiefer als die
englische. Ein Pfad, den sync.py nicht umschreibt, faellt sonst erst im
Browser auf.

Hintergrund Anker-Ebene: Die Anker werden aus den Ueberschriften generiert.
Wird eine Ueberschrift im Fork umformuliert, aendert sich ihr Slug — jeder
Querverweis darauf zeigt danach ins Leere, ohne dass eine Datei fehlt. Der
Browser springt dann kommentarlos an den Seitenanfang.
"""
import posixpath
import re
import sys
from pathlib import Path

SITE = Path(__file__).resolve().parent / "site"

IMG_RE = re.compile(r'<img[^>]*\ssrc="([^"]+)"', re.I)
HREF_RE = re.compile(r'<a[^>]*\shref="([^"]+)"', re.I)
ID_RE = re.compile(r'\sid="([^"]+)"', re.I)
NAME_RE = re.compile(r'<a[^>]*\sname="([^"]+)"', re.I)

EXTERNAL = ("http://", "https://", "data:", "mailto:", "tel:", "//")


def page_url(html: Path) -> str:
    rel = str(html.relative_to(SITE)).replace("\\", "/")
    return rel[: -len("index.html")] if rel.endswith("index.html") else rel


def resolve(page: str, ref: str) -> Path | None:
    """Zielpfad im site/-Baum, oder None wenn extern/reiner Anker."""
    ref = ref.split("#", 1)[0].split("?", 1)[0]
    if not ref or ref.startswith(EXTERNAL):
        return None
    base = ref.lstrip("/") if ref.startswith("/") else posixpath.join(page, ref)
    target = SITE / posixpath.normpath(base)
    return target / "index.html" if ref.endswith("/") else target


def main() -> int:
    if not SITE.is_dir():
        sys.exit("site/ fehlt — erst bauen.")

    pages = sorted(SITE.rglob("*.html"))
    text_of = {p: p.read_text(encoding="utf-8") for p in pages}
    # Alle vergebenen Anker je Seite (Ueberschriften-IDs und alte <a name=...>)
    anchors = {p: set(ID_RE.findall(t)) | set(NAME_RE.findall(t))
               for p, t in text_of.items()}

    broken, checked = [], 0
    anchor_broken, anchor_checked = [], 0

    for html in pages:
        page = page_url(html)
        text = text_of[html]

        for kind, pattern in (("img", IMG_RE), ("link", HREF_RE)):
            for ref in pattern.findall(text):
                target = resolve(page, ref)
                if target is None:
                    continue
                checked += 1
                if not target.exists():
                    broken.append((page or "/", kind, ref))

        for ref in HREF_RE.findall(text):
            if ref.startswith(EXTERNAL) or "#" not in ref:
                continue
            base, frag = ref.split("#", 1)
            if not frag:
                continue
            target = html if base == "" else resolve(page, base)
            # Fehlende Zieldatei meldet bereits die Datei-Ebene oben
            if target is None or not target.exists():
                continue
            anchor_checked += 1
            if frag not in anchors.get(target, set()):
                anchor_broken.append((page or "/", ref))

    if broken:
        print(f"\n{len(broken)} kaputte Verweise:", file=sys.stderr)
        for page, kind, ref in broken:
            print(f"  [{kind}] /{page}  ->  {ref}", file=sys.stderr)
    if anchor_broken:
        print(f"\n{len(anchor_broken)} tote Anker (Ziel existiert, "
              f"Ueberschrift nicht):", file=sys.stderr)
        for page, ref in anchor_broken:
            print(f"  [anchor] /{page}  ->  {ref}", file=sys.stderr)
    if broken or anchor_broken:
        return 1

    print(f"Linkcheck: {checked} interne Verweise, alle aufloesbar.")
    print(f"Ankercheck: {anchor_checked} Fragmentlinks, alle vorhanden.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
