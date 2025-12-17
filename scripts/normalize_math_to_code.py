"""
Normalize math blocks to avoid MDX acorn parsing issues:
- Convert $$...$$ (display math) into fenced code blocks ```latex ... ```
- Convert inline $...$ into inline code `...`
- During conversion we remove excessive backslashes introduced by previous escaping
  and collapse repeated braces so the math content becomes plain LaTeX text inside code blocks.

This is a pragmatic step so the site builds; it preserves the math content as readable
code blocks (not rendered math). If you'd prefer proper math rendering, we can add
`remark-math` and `rehype-katex` to the Docusaurus pipeline instead.
"""

import re
from pathlib import Path

DOCS_DIR = Path('frontend/docs/book')
if not DOCS_DIR.exists():
    print('docs/book not found')
    raise SystemExit(1)


def clean_math_inner(s: str) -> str:
    # Remove backslashes inserted before braces and other backslashes
    s = s.replace('\\', '')
    s = s.replace('\{', '{').replace('\}', '}')
    # Collapse repeated braces
    s = re.sub(r'\{+', '{', s)
    s = re.sub(r'\}+', '}', s)
    return s


def convert_file(path: Path) -> bool:
    text = path.read_text(encoding='utf-8')
    orig = text

    # Split into frontmatter and body if frontmatter exists
    front = ''
    body = text
    if text.startswith('---'):
        m = re.match(r'^(---\n[\s\S]*?\n---\n?)([\s\S]*)', text)
        if m:
            front = m.group(1)
            body = m.group(2)

    # Avoid transforming inside code fences: we'll parse manually
    parts = re.split(r'(```[\s\S]*?```)', body)
    for i, part in enumerate(parts):
        if part.startswith('```'):
            continue
        # Handle display math $$...$$
        def _disp(match):
            inner = match.group(1)
            cleaned = clean_math_inner(inner)
            return '```latex\n' + cleaned.strip() + '\n```'
        part = re.sub(r'\$\$([\s\S]*?)\$\$', _disp, part)

        # Handle inline math $...$ (avoid $$)
        # We'll replace $...$ with `...`
        def _inline(match):
            inner = match.group(1)
            cleaned = clean_math_inner(inner)
            # Escape backticks inside cleaned by replacing with \`
            cleaned = cleaned.replace('`', "\\`")
            return '`' + cleaned + '`'
        # Use a simple parser to not match $$
        part = re.sub(r'(?<!\$)\$([^$\n][^$]*?)\$(?!\$)', _inline, part)

        parts[i] = part

    new_body = ''.join(parts)
    new = front + new_body
    if new != orig:
        path.write_text(new, encoding='utf-8')
        return True
    return False


files = sorted(list(DOCS_DIR.rglob('*.md')) + list(DOCS_DIR.rglob('*.mdx')))
print('Found', len(files), 'files')
changed = 0
for f in files:
    try:
        if convert_file(f):
            print('Converted:', f)
            changed += 1
    except Exception as e:
        print('Error', f, e)

print('Total changed:', changed)
