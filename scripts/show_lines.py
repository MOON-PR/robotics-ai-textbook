import sys
from pathlib import Path
if len(sys.argv)<4:
    print('Usage: show_lines.py <file> <start> <end>')
    sys.exit(1)
path=Path(sys.argv[1])
start=int(sys.argv[2])
end=int(sys.argv[3])
text=path.read_text(encoding='utf-8').splitlines()
for i in range(start-1, min(end, len(text))):
    print(f"{i+1:4}: {text[i]}")
