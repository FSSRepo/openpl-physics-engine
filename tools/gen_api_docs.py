#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_api_docs.py — generates the Markdown API reference (ES + EN) for OpenPL.

Sources
  src/PL/pl10.h              -> function signatures + PL_* constants
  tools/api_catalog/*.yml    -> localized descriptions (es/en), one file per group
  tools/api_groups.yml       -> logical grouping of the API

Run:  python tools/gen_api_docs.py
"""
import os
import re
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HEADER = os.path.join(ROOT, "src", "PL", "pl10.h")
CATALOG = os.path.join(ROOT, "tools", "api_catalog")
GROUPS = os.path.join(ROOT, "tools", "api_groups.yml")
OUT = os.path.join(ROOT, "docs")

LANGS = ("es", "en")
META = {
    "es": {"home": "Inicio", "api": "Referencia API"},
    "en": {"home": "Home", "api": "API Reference"},
}
UI = {
    "es": {
        "params": "Parámetros", "notes": "Notas", "errors": "Errores",
        "code": "Código", "when": "Cuándo ocurre", "name": "Nombre",
        "no_params": "_Sin parámetros._", "modules": "La API se organiza en los siguientes módulos.",
        "constants": "Constantes", "const_intro": "Constantes `PL_*` aceptadas por la API, agrupadas por área.",
        "module": "Módulo", "value": "Valor",
    },
    "en": {
        "params": "Parameters", "notes": "Notes", "errors": "Errors",
        "code": "Code", "when": "When it happens", "name": "Name",
        "no_params": "_No parameters._", "modules": "The API is organized into the following modules.",
        "constants": "Constants", "const_intro": "`PL_*` constants accepted by the API, grouped by area.",
        "module": "Module", "value": "Value",
    },
}


def parse_header():
    text = open(HEADER, encoding="utf-8").read()
    funcs = {}
    pat = re.compile(r"PL_API\s+([^;\n]+?)\s+(pl\w+)\s*\(([^;]*?)\)\s*;", re.S)
    for m in pat.finditer(text):
        ret = " ".join(m.group(1).split())
        name = m.group(2)
        raw = " ".join(m.group(3).split())
        params = []
        if raw and raw != "void":
            for p in [x.strip() for x in raw.split(",") if x.strip()]:
                parts = p.rsplit(" ", 1)
                if len(parts) == 2:
                    params.append({"type": parts[0].strip(), "name": parts[1].strip().lstrip("*")})
                else:
                    params.append({"type": "", "name": p})
        funcs[name] = {"ret": ret, "params": params}

    consts, section = [], "General"
    for line in text.splitlines():
        s = line.strip()
        c = re.match(r"/\*(.*?)\*/", s)
        if c and "*/" in s:
            section = c.group(1).strip()
            continue
        if s.startswith("//"):
            section = s.lstrip("/").strip()
            continue
        d = re.match(r"#define\s+(PL_[A-Za-z0-9_]+)\s+([^\s/]+)", s)
        if d and d.group(1) != "PL_API":
            consts.append({"name": d.group(1), "value": d.group(2), "section": section})
    return funcs, consts


def L(d, lang):
    """localized value with fallback es -> value."""
    if not isinstance(d, dict):
        return d or ""
    v = d.get(lang) or d.get("es") or ""
    if isinstance(v, list):
        return v
    return v


def write(rel, content):
    path = os.path.join(OUT, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(content)


def render_function(name, sig, entry, lang):
    u = UI[lang]
    out = ["## `%s`" % name, ""]
    brief = L(entry.get("brief", {}), lang)
    if brief:
        out += [brief, ""]
    ret = sig["ret"]
    args = ", ".join(("%s %s" % (p["type"], p["name"])).strip() for p in sig["params"])
    out += ["```c", "%s %s(%s);" % (ret, name, args), "```", ""]

    params = entry.get("params") or {}
    if params:
        out += ["### %s" % u["params"], "", "| %s | |" % u["name"], "| --- | --- |"]
        for key, p in params.items():
            pname = p.get("name_" + lang) or p.get("name_es") or key
            out.append("| `%s` | %s |" % (pname, L(p, lang)))
        out.append("")
    elif sig["params"]:
        out += ["### %s" % u["params"], "", "| %s | |" % u["name"], "| --- | --- |"]
        for p in sig["params"]:
            out.append("| `%s` | %s |" % (p["name"], p["type"]))
        out.append("")

    notes = L(entry.get("notes", {}), lang) or []
    if notes:
        out += ["### %s" % u["notes"], "", "{: .note }"]
        out += ["- %s" % n for n in notes]
        out.append("")

    errors = entry.get("errors") or []
    if errors:
        out += ["### %s" % u["errors"], "", "| %s | %s |" % (u["code"], u["when"]), "| --- | --- |"]
        for e in errors:
            code = e.get("code") or "—"
            out.append("| `%s` | %s |" % (code, L(e, lang)))
        out.append("")
    return "\n".join(out)


def load_catalog():
    """Merge every tools/api_catalog/*.yml into a single {function: entry} dict."""
    catalog = {}
    if os.path.isdir(CATALOG):
        for name in sorted(os.listdir(CATALOG)):
            if not name.endswith((".yml", ".yaml")):
                continue
            with open(os.path.join(CATALOG, name), encoding="utf-8") as f:
                data = (yaml.safe_load(f) or {}).get("functions", {}) or {}
            catalog.update(data)
    return catalog


def main():
    funcs, consts = parse_header()
    catalog = load_catalog()
    groups = yaml.safe_load(open(GROUPS, encoding="utf-8"))["groups"]

    # sanity report
    grouped = {f for g in groups for f in g["functions"]}
    missing = [f for f in funcs if f not in grouped]
    if missing:
        print("WARNING: header functions not grouped:", ", ".join(missing))

    for lang in LANGS:
        m = META[lang]
        # API index
        lines = ["---", "title: %s" % m["api"], "parent: %s" % m["home"],
                 "has_children: true", "nav_order: 2", "---", "",
                 "# %s" % m["api"], "", UI[lang]["modules"], "",
                 "| %s | |" % UI[lang]["module"], "| --- | --- |"]
        for g in sorted(groups, key=lambda x: x["order"]):
            lines.append("| [%s](%s.html) | `%s` |" % (L(g["title"], lang), g["id"], g["id"]))
        lines += ["| [%s](constants.html) | `PL_*` |" % UI[lang]["constants"]]
        write("%s/api/index.md" % lang, "\n".join(lines) + "\n")

        # group pages
        for g in groups:
            body = ["---", "title: %s" % L(g["title"], lang), "parent: %s" % m["api"],
                    "grand_parent: %s" % m["home"], "nav_order: %d" % g["order"], "---", "",
                    "# %s" % L(g["title"], lang), "", '<div class="openpl-api" markdown="1">', ""]
            for fn in g["functions"]:
                if fn not in funcs:
                    print("WARNING: %s in group %s but not in header" % (fn, g["id"]))
                    continue
                body.append(render_function(fn, funcs[fn], catalog.get(fn, {}), lang))
            body += ["</div>", ""]
            write("%s/api/%s.md" % (lang, g["id"]), "\n".join(body))

        # constants page
        secs = {}
        for c in consts:
            secs.setdefault(c["section"], []).append(c)
        lines = ["---", "title: %s" % UI[lang]["constants"], "parent: %s" % m["api"],
                 "grand_parent: %s" % m["home"], "nav_order: 99", "---", "",
                 "# %s" % UI[lang]["constants"], "", UI[lang]["const_intro"], ""]
        lines += ["| %s | %s |" % (UI[lang]["name"], UI[lang]["value"]), "| --- | --- |"]
        for sec, items in secs.items():
            for c in items:
                lines.append("| `%s` | `%s` |" % (c["name"], c["value"]))
        write("%s/api/constants.md" % lang, "\n".join(lines) + "\n")

    print("Generated API docs for:", ", ".join(LANGS))


if __name__ == "__main__":
    main()
