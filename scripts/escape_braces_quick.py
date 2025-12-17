import sys
from pathlib import Path

def escape_braces(text):
    out = []
    in_fence = False
    fence_char = None
    in_backtick = False
    i = 0
    while i < len(text):
        ch = text[i]
        # detect fence start (```)
        if not in_fence and text.startswith('```', i):
            in_fence = True
            fence_char = '```'
            out.append('```')
            i += 3
            continue
        if in_fence and text.startswith('```', i):
            in_fence = False
            fence_char = None
            out.append('```')
            i += 3
            continue
        # handle inline backticks (simple heuristic)
        if not in_fence and ch == '`':
            in_backtick = not in_backtick
            out.append(ch)
            i += 1
            continue
        if not in_fence and not in_backtick and ch == '{':
            out.append('\\{')
            i += 1
            continue
        if not in_fence and not in_backtick and ch == '}':
            out.append('\\}')
            i += 1
            continue
        out.append(ch)
        i += 1
    return ''.join(out)


def process_file(path):
    p = Path(path)
    text = p.read_text(encoding='utf-8')
    new = escape_braces(text)
    if new != text:
        p.write_text(new, encoding='utf-8')
        print(f'Modified: {path}')
    else:
        print(f'No changes: {path}')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: escape_braces_quick.py <file1> [file2 ...]')
        sys.exit(1)
    for f in sys.argv[1:]:
        process_file(f)
