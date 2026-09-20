# inhero-docs — docs.inhero.de

Baut die Inhero-MR-2-Dokumentation aus dem MeshCore-Fork
([liekmarflow/MeshCore](https://github.com/liekmarflow/MeshCore),
`variants/inhero_mr2/docs/`) als indexierbare Doku-Site mit
MkDocs Material. Deutsch unter `/`, Englisch unter `/en/`,
seitengenaue hreflang-Alternates, Sitemap, robots.txt.

Der Fork bleibt Single Source of Truth — dieses Repo enthält **keine
Doku-Inhalte**, nur den Build. `sync.py` holt die Markdown-Dateien bei
jedem Build frisch aus dem Fork.

## Funktionsweise

```
sync.py        Sparse-Checkout des Forks, kopiert docs/ → docs/en und
               docs/de → docs/de, README.md → index.md, entfernt die
               🇬🇧/🇩🇪-Umschaltzeilen (ersetzt durch den Language-Switcher),
               kopiert Bilder und static/ in beide Sprachordner
mkdocs.yml     Material-Theme, mkdocs-static-i18n (DE default, EN),
               Unicode-Slugs (GitHub-kompatible Anker), Inhero-Farbe
postprocess.py macht die hreflang-Links absolut, ergänzt x-default (→ DE)
build.sh       führt alles aus; Ergebnis in site/
```

Lokal testen: `./build.sh && mkdocs serve` (serve zeigt den Stand ohne
Postprocessing — für die reine Inhaltskontrolle ausreichend).

## Deployment auf Cloudflare Pages

**1. Dieses Repo auf GitHub anlegen** (z. B. `liekmarflow/inhero-docs`,
privat genügt) und den Inhalt pushen.

**2. Cloudflare Pages Projekt anlegen:**
Cloudflare Dashboard → Workers & Pages → Create → Pages →
Connect to Git → `inhero-docs` auswählen.

| Einstellung           | Wert        |
| --------------------- | ----------- |
| Build command         | `./build.sh` |
| Build output directory| `site`      |
| Root directory        | `/`         |

Keine Umgebungsvariablen nötig; `.python-version` pinnt Python 3.12.
Erster Build läuft automatisch, Ergebnis unter `<projekt>.pages.dev`
prüfen.

**3. Custom Domain:**
Pages-Projekt → Custom domains → `docs.inhero.de` hinzufügen.
Cloudflare zeigt den nötigen CNAME an:

```
docs  CNAME  <projekt>.pages.dev
```

Den Eintrag im DNS von inhero.de setzen (liegt die Zone nicht bei
Cloudflare, funktioniert die Verifizierung ebenfalls per CNAME — den
bisherigen Redirect/Eintrag für `docs` vorher entfernen). TLS stellt
Cloudflare automatisch aus.

**4. Auto-Rebuild bei Doku-Änderungen:**
Pages-Projekt → Settings → Builds & deployments → Deploy hooks →
Hook anlegen, URL kopieren. Dann im **MeshCore-Fork**:

- Datei `meshcore-fork-workflow/trigger-docs-rebuild.yml` aus diesem
  Repo nach `.github/workflows/trigger-docs-rebuild.yml` kopieren
- Fork → Settings → Secrets and variables → Actions → neues Secret
  `CLOUDFLARE_DEPLOY_HOOK` mit der Hook-URL

Ab dann baut jeder Push, der `variants/inhero_mr2/docs/**` berührt,
die Site neu. Push in dieses Repo (Build-Konfiguration) triggert
ohnehin.

**5. Nach dem Livegang:**

- Google Search Console: Property `docs.inhero.de` anlegen (DNS-Verify
  oder über die bestehende Domain-Property), Sitemap
  `https://docs.inhero.de/sitemap.xml` einreichen
- Die Doku-Links im Shop (MR-2-Produktseite: „Vollständige
  Dokumentation: docs.inhero.de") funktionieren unverändert weiter —
  sie zeigen jetzt auf die eigene Site statt auf GitHub
- GitHub-Blob-Ansichten bleiben `noindex` → kein Duplicate Content;
  Canonical liegt auf docs.inhero.de

## Wartung

- Neue Doku-Datei im Fork? In `mkdocs.yml` unter `nav:` eintragen
  (deutscher Titel) und die englische Übersetzung unter
  `nav_translations:` ergänzen.
- Versionen sind in `requirements.txt` auf Major-Ranges gepinnt
  (MkDocs < 2.0 — MkDocs 2.0 wird inkompatibel, nicht upgraden ohne
  Prüfung).
