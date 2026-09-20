#!/usr/bin/env bash
# Build-Skript fuer docs.inhero.de (lokal; Ergebnis site/ per FTP auf Strato)
set -euo pipefail
cd "$(dirname "$0")"

pip install -r requirements.txt --quiet || pip install -r requirements.txt --quiet --break-system-packages
python3 sync.py
mkdocs build --strict
python3 postprocess.py

# .htaccess ins Site-Root (MkDocs kopiert Dotfiles nicht)
cp deploy/htaccess site/.htaccess

# robots.txt gehoert nur ins Site-Root, nicht in jeden Sprachzweig
rm -f site/en/robots.txt

# Bild- und Linkpruefung — bricht ab, wenn ein Verweis nicht aufloest
python3 check_links.py

echo "Build fertig: site/"
