"""Reviewed Fable v5 conversion using local committed sources; no deployment."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = ROOT / "variants/inhero_mr2/docs"
REFERENCE = HERE / "fable-v5-reference"
BUILD = HERE / ".build"
STAGE = BUILD / "docs-source-v5"
OUTPUT = BUILD / "docs.inhero.de"


def module(name):
    spec = importlib.util.spec_from_file_location("fable_" + name, REFERENCE / (name + ".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    # MkDocs cleans OUTPUT; sync cleans STAGE. Generated paths must stay inside
    # this tool's .build directory, including when a directory already exists.
    if BUILD.is_symlink() or not BUILD.resolve().is_relative_to(HERE):
        raise SystemExit(f"Unsafe build directory: {BUILD}")
    for directory in (STAGE, OUTPUT):
        if directory.is_symlink() or not directory.resolve().is_relative_to(BUILD.resolve()) or directory.resolve() == BUILD.resolve():
            raise SystemExit(f"Unsafe generated directory: {directory}")
    # Invalidate the last successful package before any subsequent check can
    # fail. A stale ZIP/manifest must not look like the result of this run.
    for name in ("docs.inhero.de-sftp.zip", "docs-build-manifest.json", "docs-v5-check.json"):
        artifact = BUILD / name
        if artifact.is_symlink() or not artifact.resolve().is_relative_to(BUILD.resolve()):
            raise SystemExit(f"Unsafe build artifact: {artifact}")
        artifact.unlink(missing_ok=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if subprocess.check_output(["git", "status", "--porcelain", "--", "variants/inhero_mr2/docs"], cwd=ROOT, text=True).strip():
        raise SystemExit("Commit documentation changes before building an identified release export")
    BUILD.mkdir(parents=True, exist_ok=True)
    build_env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    sync = module("sync")
    if STAGE.exists():
        shutil.rmtree(STAGE)
    sources = []
    for language in sync.LANGS:
        # Preserve original, hand-maintained hubs and static files verbatim.
        shutil.copytree(REFERENCE / "content" / language, STAGE / language)
        source_dir = SOURCE / "de" if language == "de" else SOURCE
        product_dir = STAGE / language / "mr2"
        product_dir.mkdir()
        for source in sorted(source_dir.glob("*.md")):
            body = sync.transform_markdown(source.read_text(encoding="utf-8"))
            stray = sorted(set(re.findall(r'(?:\]\(|src=["\'])(\.\./[^)"\']*)', body)))
            if stray:
                raise SystemExit(f"Unresolved relative paths in {source.name}: {stray}")
            (product_dir / sync.slug_for(source.name)).write_text(body, encoding="utf-8")
            sources.append(source)
        shutil.copytree(SOURCE / "img", product_dir / "img")
        shutil.copytree(REFERENCE / "static", STAGE / language, dirs_exist_ok=True)
    sources += [p for p in (SOURCE / "img").rglob("*") if p.is_file()]
    subprocess.run([sys.executable, "-m", "mkdocs", "build", "--strict", "-f", str(HERE / "mkdocs-docs.yml")], cwd=ROOT, env=build_env, check=True)
    postprocess = module("postprocess")
    postprocess.SITE = OUTPUT
    postprocess.main()
    # i18n's shared 404 inherits the last page's language/alternates. It is
    # served at arbitrary URLs, so remove these misleading SEO links.
    error_page = OUTPUT / "404.html"
    body = error_page.read_text(encoding="utf-8")
    body = re.sub(r'<link\b[^>]*\brel="(?:alternate|canonical)"[^>]*>\s*', '', body)
    body = body.replace('<html lang="en"', '<html lang="de"')
    body = re.sub(r'(<a\b[^>]*href=")[^"]*("[^>]*hreflang="de"[^>]*>)', r'\1/\2', body)
    body = re.sub(r'(<a\b[^>]*href=")[^"]*("[^>]*hreflang="en"[^>]*>)', r'\1/en/\2', body)
    body = re.sub(r'(<h1[^>]*>)404 - Not found(</h1>)', r'\g<1>404 – Seite nicht gefunden / Page not found\2', body)
    body = body.replace('</head>', '<meta name="robots" content="noindex">\n</head>')
    error_page.write_text(body, encoding="utf-8")
    shutil.copy2(REFERENCE / "deploy/htaccess", OUTPUT / ".htaccess")
    (OUTPUT / "en/robots.txt").unlink(missing_ok=True)
    original_check = module("check_links")
    original_check.SITE = OUTPUT
    if original_check.main() != 0:
        raise SystemExit("Original Fable v5 link checker failed")
    subprocess.run([sys.executable, str(HERE / "check_docs_v5.py")], cwd=ROOT, env=build_env, check=True)
    archive = BUILD / "docs.inhero.de-sftp.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zipped:
        for file in sorted(OUTPUT.rglob("*")):
            if file.is_file():
                zipped.write(file, file.relative_to(OUTPUT).as_posix())
    with zipfile.ZipFile(archive) as zipped:
        expected = {file.relative_to(OUTPUT).as_posix() for file in OUTPUT.rglob("*") if file.is_file()}
        assert set(zipped.namelist()) == expected and zipped.testzip() is None
        assert ".htaccess" in expected and "robots.txt" in expected and "en/robots.txt" not in expected
        assert all(zipped.read(name) == (OUTPUT / name).read_bytes() for name in expected)
    manifest = {"source_commit": commit, "site_url": "https://docs.inhero.de/", "deployment": "manual SFTP by owner",
                "conversion": "original Fable v5 transforms, hubs, theme, assets and postprocessing; corrected shared 404 metadata and search parent deduplication",
                "archive": archive.name, "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                "source_files": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(sources))},
                "conversion_files": {p.relative_to(HERE).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in sorted(REFERENCE.rglob("*")) if p.is_file() and "__pycache__" not in p.parts},
                "wrapper_files": {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                                  for name in ("build_docs.py", "build_docs_v5.py", "mkdocs-docs.yml", "check_docs_v5.py", "docs-requirements.txt", "README.md", ".gitignore", ".gitattributes")}}
    (BUILD / "docs-build-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"SFTP export ready: {OUTPUT}\nArchive: {archive}")


if __name__ == "__main__":
    sys.dont_write_bytecode = True
    sys.stdout.reconfigure(encoding="utf-8")
    main()
