"""Read-only Fable v5 export checks; writes only .build/docs-v5-check.json.

Run with the MkDocs build environment. Archive equality is checked by the
builder after this script succeeds and after the new ZIP has been created.
"""
from collections import Counter
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urljoin, urlsplit
import gzip
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
BUILD = HERE / ".build"
SITE = BUILD / "docs.inhero.de"
REFERENCE = HERE / "fable-v5-reference"
STAGE = BUILD / "docs-source-v5"
SOURCE = HERE.parents[1] / "variants/inhero_mr2/docs"
BASE = "https://docs.inhero.de/"
SLUGS = ("", "battery-guide/", "cli-cheat-sheet/", "datasheet/", "faq/",
         "power-management/", "quick-start/", "telemetry/")
ROUTES = {"", "en/"} | {prefix + "mr2/" + slug
                               for prefix in ("", "en/") for slug in SLUGS}
FEATURES = {"navigation.sections", "navigation.expand", "navigation.tracking",
            "navigation.top", "toc.follow", "search.suggest", "search.highlight",
            "content.code.copy"}


class Page(HTMLParser):
    def __init__(self, text):
        super().__init__(convert_charrefs=True)
        self.ids, self.refs, self.canonical, self.alternates = set(), [], [], []
        self.lang, self.description, self.robots = None, None, []
        self.article, self.article_refs, self.config_text = [], [], []
        self.in_article = self.skip_permalink = self.in_config = False
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "html":
            self.lang = attrs.get("lang")
        if "id" in attrs:
            self.ids.add(attrs["id"])
        if tag == "a" and "name" in attrs:
            self.ids.add(attrs["name"])
        if tag == "article":
            self.in_article = True
        if tag == "a" and "headerlink" in attrs.get("class", "").split():
            self.skip_permalink = True
        if tag == "script" and attrs.get("id") == "__config":
            self.in_config = True
        if tag == "meta":
            if attrs.get("name") == "description":
                self.description = attrs.get("content")
            if attrs.get("name") == "robots":
                self.robots.extend(attrs.get("content", "").lower().split(","))
        if tag == "link":
            if attrs.get("rel") == "canonical":
                self.canonical.append(attrs.get("href"))
            if attrs.get("rel") == "alternate":
                self.alternates.append((attrs.get("hreflang"), attrs.get("href")))
        for attribute in ("href", "src", "poster"):
            if attribute in attrs:
                self.refs.append(attrs[attribute])
                if self.in_article and not self.skip_permalink:
                    self.article_refs.append(attrs[attribute])

    def handle_endtag(self, tag):
        if tag == "article":
            self.in_article = False
        if tag == "a":
            self.skip_permalink = False
        if tag == "script":
            self.in_config = False

    def handle_data(self, data):
        if self.in_article and not self.skip_permalink:
            self.article.append(data)
        if self.in_config:
            self.config_text.append(data)

    def article_text(self):
        return " ".join(" ".join(self.article).split())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    errors, counts = [], Counter()
    hashes = {}

    def check(condition, message):
        counts["assertions"] += 1
        if not condition:
            errors.append(message)

    def equal_files(actual, expected, label):
        ok = actual.is_file() and expected.is_file()
        check(ok, f"{label}: missing {actual} or {expected}")
        if ok:
            check(actual.read_bytes() == expected.read_bytes(), f"{label}: bytes differ")
            hashes[label] = digest(actual)

    pages = {f.relative_to(SITE).as_posix(): Page(f.read_text(encoding="utf-8"))
             for f in SITE.rglob("*.html")}
    expected_files = {route + "index.html" for route in ROUTES} | {"404.html"}
    check(set(pages) == expected_files,
          f"HTML page set differs: missing={sorted(expected_files - set(pages))}, extra={sorted(set(pages) - expected_files)}")
    counts["html_pages"] = len(pages)

    def local_target(reference, page_url, label):
        absolute = urljoin(page_url, reference)
        url = urlsplit(absolute)
        if url.scheme not in ("http", "https") or url.netloc != "docs.inhero.de":
            return None
        check(url.scheme == "https", f"{label}: insecure same-host URL {reference}")
        target = (SITE / unquote(url.path).lstrip("/")).resolve()
        if target.is_dir() or url.path.endswith("/"):
            target /= "index.html"
        if not target.is_relative_to(SITE.resolve()):
            check(False, f"{label}: path escapes export: {reference}")
            return None
        check(target.is_file(), f"{label}: missing local target {reference}")
        if target.is_file() and url.fragment and target.suffix == ".html":
            target_page = pages.get(target.relative_to(SITE).as_posix())
            check(target_page is not None and unquote(url.fragment) in target_page.ids,
                  f"{label}: missing anchor {reference}")
            counts["fragment_links"] += 1
        counts["local_links"] += 1
        return absolute

    for name, page in pages.items():
        route = name.removesuffix("index.html")
        page_url = BASE + route
        for ref in page.refs:
            local_target(ref, page_url, name)
        if name == "404.html":
            check(page.lang == "de", "404: default language must be de")
            check(not page.canonical and not page.alternates, "404: canonical/alternates must be absent")
            check("noindex" in {r.strip() for r in page.robots}, "404: noindex missing")
            for ref in page.refs:
                url = urlsplit(ref)
                if not url.scheme and not url.netloc and url.path:
                    check(url.path.startswith("/"), f"404: relative URL fails on nested missing URLs: {ref}")
            continue
        language = "en" if route.startswith("en/") else "de"
        de_route = route.removeprefix("en/")
        expected_alts = {"de": BASE + de_route, "en": BASE + "en/" + de_route,
                         "x-default": BASE + de_route}
        check(page.lang == language, f"{name}: wrong html language")
        check(page.canonical == [page_url], f"{name}: wrong/duplicate canonical {page.canonical}")
        check(len(page.alternates) == 3 and dict(page.alternates) == expected_alts,
              f"{name}: wrong/duplicate hreflang {page.alternates}")
        check(bool(page.description), f"{name}: empty description")
        if language == "en":
            check(bool(page.description) and page.description.startswith("Technical documentation"),
                  f"{name}: English description missing")
        config = json.loads("".join(page.config_text) or "{}")
        check(set(config.get("features", [])) == FEATURES, f"{name}: v5 theme features differ")
        counts["content_pages"] += 1

    equal_files(SITE / ".htaccess", REFERENCE / "deploy/htaccess", "htaccess")
    equal_files(SITE / "robots.txt", REFERENCE / "static/robots.txt", "robots")
    check(not (SITE / "en/robots.txt").exists(), "robots must exist only in root")
    if (SITE / "robots.txt").is_file():
        check("Sitemap: " + BASE + "sitemap.xml" in (SITE / "robots.txt").read_text(), "robots: wrong sitemap")
    for prefix in ("", "en/"):
        equal_files(SITE / prefix / "stylesheets/inhero.css",
                    REFERENCE / "static/stylesheets/inhero.css", prefix + "inhero.css")
        for original in sorted((SOURCE / "img").iterdir()):
            if original.is_file():
                equal_files(SITE / prefix / "mr2/img" / original.name, original,
                            prefix + "mr2/img/" + original.name)
                counts["legacy_images"] += 1

    rules = []
    if (SITE / ".htaccess").is_file():
        for line in (SITE / ".htaccess").read_text(encoding="utf-8").splitlines():
            match = re.match(r"RedirectMatch\s+301\s+(\S+)\s+(\S+)", line)
            if match:
                rules.append(match.groups())
    legacy = {"QUICK_START": "quick-start", "DATASHEET": "datasheet", "BATTERY_GUIDE": "battery-guide",
              "POWER_MANAGEMENT": "power-management", "CLI_CHEAT_SHEET": "cli-cheat-sheet",
              "TELEMETRY": "telemetry", "FAQ": "faq"}
    cases = [("/" + prefix + old + ending, "/" + prefix + "mr2/" + new + "/")
             for prefix in ("", "en/") for old, new in legacy.items() for ending in ("", "/")]
    cases += [("/" + prefix + "mr-2/" + slug, "/" + prefix + "mr2/" + slug)
              for prefix in ("", "en/") for slug in SLUGS]
    for old, expected in cases:
        targets = [re.sub(pattern, re.sub(r"\$(\d+)", r"\\g<\1>", replacement), old)
                   for pattern, replacement in rules if re.search(pattern, old)]
        check(targets == [expected], f"Legacy redirect {old}: {targets}, expected {expected}")
        local_target(expected, BASE, "Legacy redirect " + old)
        counts["legacy_redirect_cases"] += 1

    xml_bytes = (SITE / "sitemap.xml").read_bytes()
    check(gzip.decompress((SITE / "sitemap.xml.gz").read_bytes()) == xml_bytes,
          "sitemap.xml.gz differs from sitemap.xml")
    tree = ET.fromstring(xml_bytes)
    ns = {"s": "http://www.sitemaps.org/schemas/sitemap/0.9", "x": "http://www.w3.org/1999/xhtml"}
    urls = tree.findall("s:url", ns)
    locations = [node.findtext("s:loc", namespaces=ns) for node in urls]
    check(len(locations) == 18 and set(locations) == {BASE + route for route in ROUTES},
          "Sitemap must contain exactly the 18 canonical content URLs")
    for node, location in zip(urls, locations):
        route = location.removeprefix(BASE).removeprefix("en/")
        expected = {"de": BASE + route, "en": BASE + "en/" + route}
        alternates = [(a.get("hreflang"), a.get("href")) for a in node.findall("x:link", ns)]
        check(len(alternates) == 2 and dict(alternates) == expected,
              f"Sitemap: incorrect language pair for {location}")
    counts["sitemap_urls"] = len(locations)

    search = json.loads((SITE / "search/search_index.json").read_text(encoding="utf-8"))
    check(set(search.get("config", {}).get("lang", [])) == {"de", "en"}, "Search languages must be de+en")
    indexed_routes = set()
    parent_routes = Counter()
    for entry in search.get("docs", []):
        location = entry.get("location", "")
        absolute = local_target(location, BASE, "Search index")
        check(absolute is not None, f"Search entry is external: {location}")
        if absolute:
            search_url = urlsplit(absolute)
            search_route = search_url.path.lstrip("/").removesuffix("index.html")
            indexed_routes.add(search_route)
            if not search_url.fragment:
                parent_routes[search_route] += 1
                check(bool(str(entry.get("title") or "").strip()),
                      f"Search parent has no title: {location}")
        counts["search_entries"] += 1
    check(indexed_routes == ROUTES, f"Search page coverage differs: {sorted(indexed_routes ^ ROUTES)}")
    # Section hits cannot replace the page-level document: Material groups
    # results by this parent and otherwise creates a blank /undefined hit.
    check(set(parent_routes) == ROUTES,
          f"Search parent page coverage differs: missing={sorted(ROUTES - set(parent_routes))}, extra={sorted(set(parent_routes) - ROUTES)}")
    for route in sorted(ROUTES):
        check(parent_routes[route] == 1,
              f"Search page {BASE + route} needs exactly one parent entry; found {parent_routes[route]}")
    counts["search_parent_entries"] = sum(parent_routes.values())
    counts["search_parent_pages"] = len(parent_routes)

    # Independently render the simple reference hubs, then compare all visible
    # article text and links. Do not execute the reference build/sync scripts.
    import markdown
    for language, prefix in (("de", ""), ("en", "en/")):
        original = REFERENCE / "content" / language / "index.md"
        equal_files(STAGE / language / "index.md", original, language + "_hub_source")
        body = re.sub(r"\A---\s*\n.*?\n---\s*\n", "", original.read_text(encoding="utf-8"), count=1, flags=re.S)
        rendered = Page("<article>" + markdown.markdown(body, extensions=["tables", "attr_list", "md_in_html"]) + "</article>")
        actual = pages.get(prefix + "index.html")
        check(actual is not None and actual.article_text() == rendered.article_text(),
              language + " hub: complete reference article text differs")
        if actual:
            expected_refs = []
            for ref in rendered.article_refs:
                if not urlsplit(ref).scheme and ref.endswith(".md"):
                    ref = ref.removesuffix(".md") + "/"
                    ref = ref.removesuffix("index/")
                expected_refs.append(urljoin(BASE + prefix, ref))
            actual_refs = [urljoin(BASE + prefix, ref) for ref in actual.article_refs]
            check(Counter(expected_refs) == Counter(actual_refs), language + " hub: reference links differ")
        counts["reference_hubs"] += 1

    report = {"status": "passed" if not errors else "failed", "site_url": BASE,
              "checks": dict(counts), "errors": errors, "verified_sha256": hashes,
              "output_sha256": {p.relative_to(SITE).as_posix(): digest(p)
                                 for p in sorted(SITE.rglob("*")) if p.is_file()},
              "scope": "Local files only; no website mutation or deployment. HTTP redirect execution and cache headers require the SFTP host. New ZIP equality is checked by the builder after packaging."}
    (BUILD / "docs-v5-check.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "checks": report["checks"], "errors": errors}, ensure_ascii=False, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.dont_write_bytecode = True
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if BUILD.is_symlink() or not BUILD.resolve().is_relative_to(HERE):
        raise SystemExit(f"Unsafe build directory: {BUILD}")
    BUILD.mkdir(parents=True, exist_ok=True)
    try:
        raise SystemExit(main())
    except Exception as error:
        report = {"status": "failed", "errors": [f"{type(error).__name__}: {error}"]}
        (BUILD / "docs-v5-check.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
