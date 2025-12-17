#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
DOCS_DIR = ROOT / 'frontend' / 'docs'


def find_code_fence_ranges(text):
    ranges = []
    # find fenced code blocks ``` or ~~~
    for m in re.finditer(r"(^|\n)(```|~~~).*?\2\s*(\n|$)", text, flags=re.S):
        start = m.start(0)
        end = m.end(0)
        ranges.append((start, end))
    return ranges


def find_backtick_spans(text):
    spans = []
    i = 0
    n = len(text)
    while i < n:
        if text[i] == '`':
            # count number of backticks
            j = i
            while j < n and text[j] == '`':
                j += 1
            bt_len = j - i
            # find next matching backtick sequence
            k = text.find('`' * bt_len, j)
            if k == -1:
                break
            spans.append((i, k + bt_len))
            i = k + bt_len
        else:
            i += 1
    return spans


def in_ranges(pos, ranges):
    for a, b in ranges:
        if a <= pos < b:
            return True
    return False


def process_file(p: Path):
    text = p.read_text(encoding='utf8')
    # skip if file likely contains MDX imports/JSX to avoid breaking real expressions
    if re.search(r"^import\s+|^export\s+|<\w", text, flags=re.M):
        return False, 'skipped (contains import/export/JSX)'

    # find frontmatter and exclude it
    fm_end = 0
    m = re.match(r'^---\n(.*?)\n---\n', text, flags=re.S)
    if m:
        fm_end = m.end()

    code_ranges = find_code_fence_ranges(text)
    backtick_spans = find_backtick_spans(text)
    ignore_ranges = code_ranges + backtick_spans

    changed = False
    chars = list(text)
    for i, ch in enumerate(chars):
        if i < fm_end:
            continue
        if ch in '{}':
            if in_ranges(i, ignore_ranges):
                continue
            # don't escape if already escaped
            if i > 0 and chars[i-1] == '\\':
                continue
            # escape
            chars[i] = '\\' + ch
            changed = True

    if changed:
        backup = p.with_suffix(p.suffix + '.bak')
        p.rename(backup)
        p.write_text(''.join(chars), encoding='utf8')
        return True, f'escaped braces, backup at {backup.name}'
    return False, 'no change'


def main():
    files = sorted(list(DOCS_DIR.rglob('*.md')) + list(DOCS_DIR.rglob('*.mdx')))
    summary = []
    for p in files:
        ok, msg = process_file(p)
        summary.append((p.relative_to(ROOT), ok, msg))
    for s in summary:
        print(s[0], s[1], s[2])


if __name__ == '__main__':
    main()
