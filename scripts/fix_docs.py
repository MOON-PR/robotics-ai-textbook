#!/usr/bin/env python3
import os
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS_DIR = ROOT / 'frontend' / 'docs'
SIDEBARS_PATH = ROOT / 'frontend' / 'sidebars.js'
INDEX_PAGE = ROOT / 'frontend' / 'src' / 'pages' / 'index.tsx'
DOT_DOCUSSAURUS = ROOT / 'frontend' / '.docusaurus'


def slug_from_path(p: Path):
    # path relative to docs dir, with separators replaced by '-'
    rel = p.relative_to(DOCS_DIR).with_suffix('')
    parts = [part for part in rel.parts]
    return '-'.join(parts)


def title_from_file(content: str, path: Path):
    # look for first H1 or H2
    for line in content.splitlines():
        m = re.match(r'^(#{1,2})\s+(.*)$', line)
        if m:
            return m.group(2).strip()
    # fallback to filename
    name = path.stem
    name = re.sub(r'^\d+[-_]?','', name)
    return ' '.join([w.capitalize() for w in name.split('-')])


def ensure_frontmatter(path: Path):
    text = path.read_text(encoding='utf8')
    fm = {}
    m = re.match(r'^---\n(.*?)\n---\n', text, flags=re.S)
    if m:
        block = m.group(1)
        for line in block.splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                fm[k.strip()] = v.strip()
        body = text[m.end():]
    else:
        body = text

    new_id = slug_from_path(path)
    changed = False
    if fm.get('id') != new_id:
        fm['id'] = new_id
        changed = True

    if 'title' not in fm or not fm['title']:
        fm['title'] = title_from_file(text, path)
        changed = True

    # determine sidebar_position from filename prefix if exists
    pos = None
    mnum = re.match(r'^(\d+)', path.stem)
    if mnum:
        pos = int(mnum.group(1))
    # fallback: do not set if already exists
    if 'sidebar_position' not in fm or not fm.get('sidebar_position'):
        if pos is None:
            # assign 0 as default to keep stable ordering
            fm['sidebar_position'] = '0'
        else:
            fm['sidebar_position'] = str(pos)
        changed = True

    if changed:
        # rebuild frontmatter block
        block_lines = ['---']
        # keep ordering id, title, sidebar_position
        block_lines.append(f"id: {fm['id']}")
        block_lines.append(f"title: {fm['title']}")
        block_lines.append(f"sidebar_position: {fm['sidebar_position']}")
        block_lines.append('---\n')
        new_text = '\n'.join(block_lines) + body.lstrip('\n')
        path.write_text(new_text, encoding='utf8')
    return new_id


def collect_docs():
    docs = []
    for p in sorted(DOCS_DIR.rglob('*.md')) + sorted(DOCS_DIR.rglob('*.mdx')):
        docs.append(p)
    return docs


def generate_sidebar(ids):
    # Create a simple single category sidebar
    lines = ["module.exports = {", "  docsSidebar: [", "    { type: 'category', label: 'Robotics Course', items: ["]
    for id in ids:
        lines.append(f"      '{id}',")
    lines.append("    ] },")
    lines.append("  ]");
    lines.append("};")
    content = '\n'.join(lines) + '\n'
    SIDEBARS_PATH.write_text(content, encoding='utf8')


def update_index(first_id):
    if not INDEX_PAGE.exists():
        return
    text = INDEX_PAGE.read_text(encoding='utf8')
    # replace hardcoded /docs/... occurrences to use first chapter
    new_target = f"/docs/{first_id}"
    new_text = re.sub(r"/docs/[\w\-\/]+", new_target, text)
    INDEX_PAGE.write_text(new_text, encoding='utf8')


def remove_docusaurus_cache():
    if DOT_DOCUSSAURUS.exists():
        import shutil
        try:
            shutil.rmtree(DOT_DOCUSSAURUS)
        except Exception as e:
            print(f"Warning: couldn't remove {DOT_DOCUSSAURUS}: {e}")


def main():
    docs = collect_docs()
    ids = []
    for p in docs:
        nid = ensure_frontmatter(p)
        ids.append(nid)

    # sort ids for stable ordering
    ids_sorted = sorted(ids)
    generate_sidebar(ids_sorted)
    first = ids_sorted[0] if ids_sorted else ''
    update_index(first)
    remove_docusaurus_cache()
    print('First chapter id:', first)


if __name__ == '__main__':
    main()
