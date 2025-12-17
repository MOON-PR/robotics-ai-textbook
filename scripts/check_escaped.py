from pathlib import Path
p=Path('frontend/docs')
files=sorted(list(p.rglob('*.md'))+list(p.rglob('*.mdx')))
print('Found',len(files),'files')
for f in files:
    s=f.read_text(encoding='utf-8')
    if '{\\{' in s or '\\}}' in s:
        print('Has escaped braces:',f)
