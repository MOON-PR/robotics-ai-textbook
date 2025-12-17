import re
import json
from pathlib import Path

build_log = Path('build_output.txt').read_text(encoding='utf-8', errors='replace')

errs = []
for m in re.finditer(r'Error: MDX compilation failed for file "([^"]+)"', build_log):
    file_path = m.group(1)
    # search for "line": N after this match
    rest = build_log[m.end(): m.end()+200]
    lm = re.search(r'"line"\s*:\s*(\d+)', rest)
    line_no = lm.group(1) if lm else '1'
    errs.append((file_path, int(line_no)))

if not errs:
    print('No matches found')
    exit(0)

for path, line in errs:
    print('FILE::', path)
    print('LINE::', line)
    try:
        p = Path(path)
        if p.exists():
            lines = p.read_text(encoding='utf-8', errors='replace').splitlines()
            ln = int(line)
            start = max(1, ln-5)
            end = min(len(lines), ln+5)
            for i in range(start, end+1):
                prefix = '>' if i==ln else ' '
                print(f"{prefix} {i:4}: {lines[i-1]}")
        else:
            print('   File not found:', path)
    except Exception as e:
        print('   Error reading file:', e)
    print('\n' + ('-'*60) + '\n')
