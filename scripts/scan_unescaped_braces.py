from pathlib import Path
import sys

def scan(path):
    text = Path(path).read_text(encoding='utf-8')
    in_fence = False
    fence_seq = '```'
    in_backtick = False
    lines = text.splitlines()
    for lineno, line in enumerate(lines, start=1):
        i=0
        while i < len(line):
            if not in_fence and line.startswith(fence_seq, i):
                in_fence = True
                i += len(fence_seq)
                continue
            if in_fence and line.startswith(fence_seq, i):
                in_fence = False
                i += len(fence_seq)
                continue
            ch = line[i]
            if not in_fence and ch == '`':
                # toggle inline backtick (simple, doesn't handle multiple backticks well)
                in_backtick = not in_backtick
                i += 1
                continue
            if not in_fence and not in_backtick and ch in '{}':
                print(f"{path}:{lineno}:{i+1}: found '{ch}' outside code fence/backtick: {line.strip()}")
                break
            i += 1

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: scan_unescaped_braces.py <file1> [file2 ...]')
        sys.exit(1)
    for p in sys.argv[1:]:
        scan(p)
