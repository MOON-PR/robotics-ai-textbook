from pathlib import Path
import re

base = Path('frontend/docs/book')
if not base.exists():
    print('frontend/docs/book not found')
    raise SystemExit(1)

files = sorted(list(base.rglob('*.md')) + list(base.rglob('*.mdx')))
missing = []
for f in files:
    txt = f.read_text(encoding='utf-8')
    # read up to first 30 lines
    head = '\n'.join(txt.splitlines()[:40])
    if not head.strip().startswith('---'):
        missing.append((f,'no-frontmatter'))
        continue
    # extract frontmatter block
    m = re.match(r'^---\n([\s\S]*?)\n---', head)
    if not m:
        missing.append((f,'bad-frontmatter'))
        continue
    fm = m.group(1)
    has_id = bool(re.search(r'^id:\s*.+$','\n'.join([ln.strip() for ln in fm.splitlines()]), re.M))
    has_title = bool(re.search(r'^title:\s*.+$','\n'.join([ln.strip() for ln in fm.splitlines()]), re.M))
    has_pos = bool(re.search(r'^sidebar_position:\s*\d+','\n'.join([ln.strip() for ln in fm.splitlines()]), re.M))
    if not (has_id and has_title and has_pos):
        missing.append((f, ('id' if not has_id else '')+' '+('title' if not has_title else '')+' '+('sidebar_position' if not has_pos else '')))

print('Checked', len(files), 'files')
if missing:
    print('Files missing frontmatter fields:')
    for f,reason in missing:
        print(f, reason)
else:
    print('All files have id, title, and sidebar_position in frontmatter')
