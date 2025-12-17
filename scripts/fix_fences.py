from pathlib import Path
import sys


def fix_file(path):
    p = Path(path)
    lines = p.read_text(encoding='utf-8').splitlines()
    changed = False
    for i, ln in enumerate(lines):
        if ln == '    ```latex':
            lines[i] = '```latex'
            changed = True
        if ln == '    ```':
            lines[i] = '```'
            changed = True
    if changed:
        p.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        print(f'Patched fences in: {path}')
    else:
        print(f'No fence changes: {path}')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: fix_fences.py <file1> [file2 ...]')
        sys.exit(1)
    for f in sys.argv[1:]:
        fix_file(f)
