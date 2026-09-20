#!/usr/bin/env python3
"""
sync.py — holt die Inhero-Hardware-Dokumentation aus den Quell-Repos und
bereitet sie fuer den MkDocs-Build auf.

Zielstruktur (je Sprache):
    /                      Hub-Startseite (aus content/, handgepflegt)
    /mr2/                  Produkt-Uebersicht (aus README.md der Quelle)
    /mr2/quick-start/      usw.

Neues Produkt aufnehmen:
    1. Eintrag in PRODUCTS ergaenzen
    2. In mkdocs.yml unter nav: den Block eintragen
    3. Auf der Hub-Startseite (content/de/index.md, content/en/index.md)
       verlinken
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CHECKOUT_BASE = ROOT / ".src"
DOCS = ROOT / "docs"
CONTENT = ROOT / "content"   # handgepflegte Seiten (Hub), je Sprache
STATIC = ROOT / "static"     # CSS, robots.txt

LANGS = ("de", "en")

# Dateiname in der Quelle -> URL-Slug im Zielverzeichnis.
# Unbekannte Dateien: kleingeschrieben, Unterstriche zu Bindestrichen.
SLUG_MAP = {
    "README.md": "index.md",          # -> /<produkt>/
    "QUICK_START.md": "quick-start.md",
    "DATASHEET.md": "datasheet.md",
    "BATTERY_GUIDE.md": "battery-guide.md",
    "POWER_MANAGEMENT.md": "power-management.md",
    "CLI_CHEAT_SHEET.md": "cli-cheat-sheet.md",
    "TELEMETRY.md": "telemetry.md",
    "FAQ.md": "faq.md",
}

PRODUCTS = [
    {
        "slug": "mr2",
        "repo": "https://github.com/liekmarflow/MeshCore.git",
        "ref": "main",
        # Pfad im Repo: EN-Dateien liegen hier, DE unter <docs_path>/<de_subdir>
        "docs_path": "variants/inhero_mr2/docs",
        "de_subdir": "de",
        "img_dir": "img",
    },
]

LANG_SWITCH_RE = re.compile(
    r"^>\s*(\U0001F1EC\U0001F1E7|\U0001F1E9\U0001F1EA).*?"
    r"\[(English [Vv]ersion|Deutsche [Vv]ersion)\]\([^)]*\)\s*$"
)


def run(cmd):
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def default_slug(name: str) -> str:
    return name[:-3].lower().replace("_", "-") + ".md"


def slug_for(name: str) -> str:
    return SLUG_MAP.get(name, default_slug(name))


def checkout(product) -> Path:
    dest = CHECKOUT_BASE / product["slug"]
    if dest.exists():
        shutil.rmtree(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    run(["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
         "--branch", product["ref"], product["repo"], str(dest)])
    run(["git", "-C", str(dest), "sparse-checkout", "set", product["docs_path"]])
    return dest / product["docs_path"]


def transform_markdown(text: str) -> str:
    """Sprachumschalt-Zeilen entfernen, Links auf die neuen Slugs umbiegen,
    Bildpfade vereinheitlichen."""
    lines = [ln for ln in text.splitlines()
             if not LANG_SWITCH_RE.match(ln.strip())]
    text = "\n".join(lines) + "\n"

    # Links auf Geschwisterdateien: FOO_BAR.md -> foo-bar.md,
    # README.md -> index.md. Anker bleiben unveraendert (Slugify ist
    # GitHub-kompatibel).
    def repl(m):
        prefix, name, frag = m.group(1) or "", m.group(2), m.group(3) or ""
        return f"({prefix}{slug_for(name)}{frag})"

    text = re.sub(r"\((\./)?([A-Za-z0-9_]+\.md)(#[^)]*)?\)", repl, text)

    # DE liegt in der Quelle eine Ebene tiefer und referenziert ../img/;
    # im Ziel liegt img/ neben den Dateien. Beide Schreibweisen abdecken:
    # Markdown ![alt](../img/x.png) UND HTML <img src="../img/x.png">.
    text = re.sub(r'(\]\(|src=["\'])\.\./img/', r'\1img/', text)
    return text


def sync_dir(src_dir: Path, dst_dir: Path):
    dst_dir.mkdir(parents=True, exist_ok=True)
    for md in sorted(src_dir.glob("*.md")):
        out = dst_dir / slug_for(md.name)
        body = transform_markdown(md.read_text(encoding="utf-8"))
        stray = sorted(set(re.findall(r'(?:\]\(|src=["\'])(\.\./[^)"\']*)', body)))
        if stray:
            sys.exit(f"FEHLER in {md.name}: unaufgeloeste Relativpfade {stray}\n"
                     f"       transform_markdown() muss diese Form abdecken.")
        out.write_text(body, encoding="utf-8")
        print(f"    {md.name} -> {out.relative_to(ROOT)}")


def main():
    if DOCS.exists():
        shutil.rmtree(DOCS)

    # 1. Handgepflegte Hub-Seiten
    for lang in LANGS:
        src = CONTENT / lang
        if not src.exists():
            sys.exit(f"FEHLT: {src} — Hub-Startseite fuer '{lang}'")
        shutil.copytree(src, DOCS / lang)
        print(f"  content/{lang}/ -> docs/{lang}/")

    # 2. Produkt-Dokumentation
    for product in PRODUCTS:
        print(f"== Produkt: {product['slug']} ==")
        src = checkout(product)
        for lang in LANGS:
            src_dir = src / product["de_subdir"] if lang == "de" else src
            dst_dir = DOCS / lang / product["slug"]
            print(f"  [{lang}]")
            sync_dir(src_dir, dst_dir)
            img_src = src / product["img_dir"]
            if img_src.exists():
                shutil.copytree(img_src, dst_dir / "img")
                print(f"    img/ -> {(dst_dir / 'img').relative_to(ROOT)}")

    # 3. Statische Assets in jeden Sprachzweig
    if STATIC.exists():
        for lang in LANGS:
            shutil.copytree(STATIC, DOCS / lang, dirs_exist_ok=True)
            print(f"  static/ -> docs/{lang}/")

    print("Sync abgeschlossen.")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)
