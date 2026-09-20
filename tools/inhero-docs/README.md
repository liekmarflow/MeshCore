# docs.inhero.de lokal bauen

Dieses Werkzeug erzeugt die deutsche und englische Inhero-MR2-Dokumentation aus
dem lokalen MeshCore-Repository als statische Website. Es verwendet das geprüfte
Fable-v5-Projekt mit dessen Hub-Seiten, Gestaltung, URL-Struktur und
Strato-Konfiguration. Der Betreiber lädt das Ergebnis selbst per SFTP hoch;
das Werkzeug veröffentlicht nichts und benötigt keine Zugangsdaten.

## Voraussetzungen

- Python 3.12 oder 3.13 und Git.
- Die Dokumentationsdateien unter `variants/inhero_mr2/docs/` müssen sauber
  eingecheckt sein. Bei lokalen oder neuen, nicht eingecheckten Dateien dort
  bricht der Build ab, damit der Quellcommit im Manifest eindeutig ist.
- Die Python-Pakete aus `docs-requirements.txt`. Für die Installation wird
  Internetzugang benötigt; der Build verwendet anschließend lokale Quellen.

Alle folgenden Befehle werden im **MeshCore-Repository-Verzeichnis** ausgeführt.
Die virtuelle Umgebung liegt im bereits ignorierten `.pio/`-Verzeichnis.

## Windows / PowerShell

```powershell
# Einmalige Einrichtung:
python -m venv .pio/docs-export-venv
./.pio/docs-export-venv/Scripts/python.exe -m pip install -r tools/inhero-docs/docs-requirements.txt

# Nach künftigen Doku-Änderungen und deren Commit:
./.pio/docs-export-venv/Scripts/python.exe tools/inhero-docs/build_docs.py
```

## Linux / macOS

```sh
# Einmalige Einrichtung:
python3 -m venv .pio/docs-export-venv
.pio/docs-export-venv/bin/python -m pip install -r tools/inhero-docs/docs-requirements.txt

# Nach künftigen Doku-Änderungen und deren Commit:
.pio/docs-export-venv/bin/python tools/inhero-docs/build_docs.py
```

Eine Aktivierung der Umgebung ist nicht erforderlich. Das Werkzeug bestimmt den
Repositorypfad aus seinem eigenen Speicherort; ein bestimmter Benutzerpfad oder
Releaseordner ist nicht hinterlegt.

## Ergebnis und SFTP

Alle erzeugten Dateien liegen unter `tools/inhero-docs/.build/`; dieses
Verzeichnis wird nicht eingecheckt:

| Pfad innerhalb von `.build/` | Zweck |
| --- | --- |
| `docs.inhero.de/` | Fertige Website einschließlich `.htaccess` |
| `docs.inhero.de-sftp.zip` | Derselbe vollständige Inhalt als ZIP, ohne übergeordneten Ordner |
| `docs-build-manifest.json` | Quellcommit, SHA-256 der Dokumentationsquellen, Originalreferenz, Werkzeugdateien und des ZIPs |
| `docs-v5-check.json` | Prüfbericht mit Dateihashes, Seiten, Links, Ankern, Bildern, Suche, Sitemap und Weiterleitungen |
| `docs-source-v5/` | Generierte MkDocs-Zwischenquellen |

Den **Inhalt** von `docs.inhero.de/` in das bestehende Dokumentenverzeichnis des
SFTP-Hosts laden. Auch die verborgene `.htaccess` übertragen. Die vorhandene
Strato-Konfiguration wird beibehalten; historische Dokumentations-URLs werden
auf die aktuellen Seiten weitergeleitet. Deutsch liegt unter `/mr2/`, Englisch
unter `/en/mr2/`; die Startseiten liegen unter `/` und `/en/`.

Nur nach einem erfolgreich abgeschlossenen Build hochladen. Der Build entfernt
das vorherige ZIP, Manifest und den Prüfbericht vorab. Bei einem Fehler kann
der Websiteordner noch alte oder unvollständige Dateien enthalten; der
Fehlerstatus des aktuellen Aufrufs ist maßgeblich.

Vor dem Hochladen kann die Website lokal geprüft werden, zum Beispiel unter
Windows:

```powershell
./.pio/docs-export-venv/Scripts/python.exe -m http.server 8766 --bind 127.0.0.1 --directory tools/inhero-docs/.build/docs.inhero.de
```

Unter Linux denselben Aufruf mit `.pio/docs-export-venv/bin/python` verwenden.
Danach `http://127.0.0.1:8766/` öffnen. Der lokale Python-Webserver führt die
Apache-/Strato-Regeln der `.htaccess` nicht aus; HTTPS-Weiterleitung und Cacheheader
lassen sich erst auf dem vorgesehenen Host prüfen.

## Prüfungen und bewusste Anpassungen

Der Build führt MkDocs mit `--strict`, den originalen Fable-Linkcheck und den
ergänzten Exportcheck aus. Er prüft insbesondere die 18 Dokumentationsseiten,
DE/EN-Hubs, bestehende Bildpfade, Sprungmarken, Sprache/Canonical/hreflang,
Suchindex mit einem Seiteneintrag pro Dokument, Sitemap und historische
Weiterleitungsziele. Anschließend wird geprüft, dass das ZIP den Websiteinhalt
vollständig und unverändert enthält.

Die unveränderte Originalreferenz liegt unter `fable-v5-reference/` (14 Dateien).
Ihre ursprüngliche README beschreibt teilweise Cloudflare Pages und einen im
Original-ZIP nicht enthaltenen Workflow. Das ist historischer Kontext;
**der aktuelle Ablauf steht in dieser README und verwendet manuellen SFTP-Upload.**
Die ursprünglichen `build.sh` und `sync.py` werden nicht direkt ausgeführt.

Die lokalen Wrapper behalten Original-Hubs, statische Dateien,
Markdown-Transformation, Theme und hreflang-Postprocessing bei. Sie verwenden
die sauber eingecheckten lokalen Produktdokumente statt eines neuen
Remote-Checkouts. Zusätzlich korrigieren sie zwei Fehler des alten Builds:

- Die gemeinsame 404-Seite erhält keine Telemetrie-hreflang-Links, einen
  zweisprachigen Titel, Sprachwahl zu den Startseiten und `noindex`.
- Die i18n-Suchdeduplizierung ist deaktiviert. DE und EN sind explizit als
  Suchsprachen gesetzt; identische Seitentitel bleiben damit in beiden Sprachen
  als Eltern der jeweiligen Abschnittstreffer erhalten.

Für normale Inhaltsänderungen die Produktdokumente im Repository bearbeiten.
Für Änderungen am Konvertierungsverhalten die lokalen Wrapper anpassen;
die mitgelieferte Originalreferenz bleibt als Vergleichsstand unverändert.
Neue Produktseiten erfordern außerdem Navigation und Prüferwartungen in
`mkdocs-docs.yml` und `check_docs_v5.py`.
