# MeshCore — Inhero MR-2 Fork: Sitzungsstand

> **Hinweis:** Dieses Repo ist als eigenständiger Produkt-Fork etabliert.
> Kein PR-Verkehr zu upstream geplant. CONTEXT.md darf hier auf `main`
> liegen, weil sie nicht in einen Upstream-PR-Diff geraten kann
> (Upstream-PRs liefen früher von Feature-Branches, die wir nicht mehr
> pflegen).

## Letzter Commit

`14a5c31c` — chore: ignore local release/ asset staging directory  
Tag: `hw1.x-v0.1` (Firmware-Release v0.1 für Hardware Rev 1.x)

## Repo-Layout (Stand 2026-04-28)

- **Branches (origin):** `main`, `gh-pages`. Mehr nicht. Alle alten
  Feature-/Backup-/Refactor-Branches gelöscht.
- **`main`** = konsolidierter Stand: Code aus
  `refactor/inhero-mr2-modularize` + Docs aus `feature/inhero-mr2-rev1.x`
  zusammengeführt. Letzter in der Praxis getesteter Stand.
- **Root-`README.md`** = Fork-spezifische Landing-Page.
  Upstream-Original liegt unter `README.upstream.md`.
- **Variant-Code:** `variants/inhero_mr2/`
- **Variant-Docs:** `variants/inhero_mr2/docs/` (EN + `de/`)
- **Release-Assets (lokal, gitignored):** `release/<tag>/`

## Build-Envs

- `Inhero_MR2_repeater`
- `Inhero_MR2_repeater_bridge_rs232`
- `Inhero_MR2_sensor`

Pro Env werden `firmware.uf2`, `firmware.zip` (DFU) und `firmware.hex`
erzeugt. UF2 entsteht via Custom-Target `create_uf2`, die ZIP fällt
beim Default-Build heraus.

## Release-Workflow (für künftige Versionen)

```powershell
Set-Location -LiteralPath c:\Users\wolfr\Documents\VS-Code-Projects\extern\MeshCore

# 1. Alle drei Envs bauen (UF2 + Default für ZIP)
foreach ($e in @("Inhero_MR2_repeater","Inhero_MR2_repeater_bridge_rs232","Inhero_MR2_sensor")) {
  & C:\Users\wolfr\.platformio\penv\Scripts\platformio.exe run -e $e -t create_uf2
  & C:\Users\wolfr\.platformio\penv\Scripts\platformio.exe run -e $e
}

# 2. Assets sammeln
$tag = "hw1.x-v0.2"   # anpassen
$dst = "release\$tag"
New-Item -ItemType Directory -Force -Path $dst | Out-Null
foreach ($e in @("Inhero_MR2_repeater","Inhero_MR2_repeater_bridge_rs232","Inhero_MR2_sensor")) {
  foreach ($x in @("uf2","hex","zip")) {
    Copy-Item ".pio\build\$e\firmware.$x" "$dst\$e-$tag.$x" -Force
  }
}
Get-FileHash -Algorithm SHA256 (Get-ChildItem $dst\*).FullName |
  Select-Object @{n="File";e={Split-Path $_.Path -Leaf}},Hash |
  Format-Table -AutoSize | Out-String -Width 200 |
  Tee-Object -FilePath "$dst\SHA256SUMS.txt"

# 3. Tag + Push
$msg = "Inhero MR-2 (Hardware Rev 1.x) firmware release v0.2`n`n..."
$msg | Out-File "$dst\tagmsg.txt" -Encoding utf8
git tag -a $tag -F "$dst\tagmsg.txt"
git push origin $tag

# 4. Release auf https://github.com/liekmarflow/MeshCore/releases/new
#    Tag aus Dropdown wählen, Assets per Drag&Drop aus $dst hochladen.
```

## Upstream-Sync (wenn er ansteht)

```powershell
git fetch upstream
git merge upstream/main
```

**Erwarteter Konflikt:** `README.md` (unsere Fork-Landing-Page vs.
upstream-MeshCore-README). Auflösung: **immer unsere Version behalten**:

```powershell
git checkout --ours README.md
git add README.md
```

Wenn upstream relevante Variant-Pattern-Änderungen hat (z. B. neue
Hooks in `nrf52_base`), die für die Inhero-Variant relevant sind:
manuell prüfen und ggf. nachpflegen. Keine automatische Übernahme von
Variant-Verzeichnissen anderer Boards nötig.

Falls upstream die Datei `README.upstream.md` selbst irgendwann
einführt: umbenennen oder mergen wie es sich ergibt.

## Stolpersteine

- **PowerShell `git tag -m … -m … -m …`:** schluckt nur erste 1–2
  Argumente, Rest fliegt mit "too many arguments" raus. Für mehrzeilige
  Tag-Messages immer `-F file` verwenden.
- **`platformio.exe ... -t create_uf2`** baut KEIN `firmware.zip`. Für
  ZIP zusätzlich `pio run -e <env>` ohne Target laufen lassen.
- **GitHub-Topics:** Space-getrennt im UI-Feld erzeugt EINEN langen Tag,
  nicht mehrere. Komma oder Enter zwischen jedem Tag verwenden.
- **Forks und CONTEXT.md:** Gilt nur als Risiko, wenn man Upstream-PRs
  fährt. Hier nicht der Fall — daher OK.

## Nächster Schritt beim Wiedereinstieg

Routinearbeit — wahrscheinlich erst wieder relevant bei:
- neuer Hardware-Rev 1.2 oder 2.x → neuer Tag (`hw1.2-v0.1` oder `hw2.x-v0.1`)
- Bugfix-Release auf Rev 1.x → `hw1.x-v0.2`
- Upstream-Sync nach größerem MeshCore-Update
