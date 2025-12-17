#!/usr/bin/env python3
"""
Fix MDX/acorn parsing errors by properly escaping LaTeX/math expressions.
In MDX, curly braces have special meaning (they denote JSX expressions).
To use literal braces in math mode, we must escape them as {\{ and \}}.
"""

import os
import re
from pathlib import Path

def fix_latex_in_file(filepath):
    """Fix LaTeX expressions in an MDX/MD file by escaping braces."""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    original_content = content
    lines = content.split('\n')
    fixed_lines = []
    in_code_block = False
    in_frontmatter = False
    
    for i, line in enumerate(lines):
        # Track code blocks (don't modify code blocks)
        if line.strip().startswith('```'):
            in_code_block = not in_code_block
            fixed_lines.append(line)
            continue
        
        if in_code_block:
            fixed_lines.append(line)
            continue
        
        # Track frontmatter
        if i == 0 and line.strip().startswith('---'):
            #!/usr/bin/env python3
            """
            Robustly escape curly braces inside LaTeX math blocks so MDX's acorn
            doesn't try to interpret braces as JS expressions.

            This script:
            - Scans all .md/.mdx files under `frontend/docs`.
            - For each file, it finds display math blocks (`$$...$$`) and inline math
              (`$...$`) and escapes all `{` -> '{\\{' and `}` -> '\\}}' inside those blocks.
            - Skips code fences and frontmatter.

            Note: This is a targeted transformation intended to avoid manual edits.
            If you prefer to use `remark-math`+`rehype-katex`, we can add those plugins
            and install dependencies instead.
            """

            import re
            from pathlib import Path


            def escape_braces_in_math(text: str) -> str:
                """Escape braces inside math segments in the provided text.

                This handles both $$...$$ (multiline) and $...$ (inline), carefully
                avoiding code fences and frontmatter.
                """

                # Escape braces in $$...$$ (DOTALL)
                def _escape(match):
                    inner = match.group(1)
                    escaped = inner.replace('{', '{\\{').replace('}', '\\}}')
                    return '$$' + escaped + '$$'

                text = re.sub(r'\$\$([\s\S]*?)\$\$', _escape, text)

                # Escape braces in inline $...$ (avoid $$ which are already handled)
                # We'll parse by walking through the string to avoid incorrect matches.
                result = []
                i = 0
                n = len(text)
                while i < n:
                    if text.startswith('```', i):
                        # Copy code fence until closing fence
                        end = text.find('```', i + 3)
                        if end == -1:
                            result.append(text[i:])
                            break
                        end += 3
                        result.append(text[i:end])
                        i = end
                        continue

                    if text[i] == '$':
                        # If double dollar, skip (already handled)
                        if i + 1 < n and text[i + 1] == '$':
                            result.append('$$')
                            i += 2
                            continue
                        # single dollar: find next single dollar
                        j = i + 1
                        while j < n:
                            if text[j] == '$':
                                break
                            # skip escaped dollar
                            if text[j] == '\\' and j + 1 < n:
                                j += 2
                                continue
                            j += 1
                        if j >= n:
                            # no closing dollar found
                            result.append(text[i:])
                            break
                        # escape inside text[i+1:j]
                        inner = text[i+1:j]
                        escaped = inner.replace('{', '{\\{').replace('}', '\\}}')
                        result.append('$' + escaped + '$')
                        i = j + 1
                        continue

                    # Otherwise, copy one char
                    result.append(text[i])
                    i += 1

                return ''.join(result)


            def fix_file(path: Path) -> bool:
                s = path.read_text(encoding='utf-8')

                # Skip frontmatter and leave it unchanged, but process body
                if s.startswith('---'):
                    # find end of frontmatter
                    fm_end = s.find('\n---', 3)
                    if fm_end != -1:
                        fm_end += 4
                        front = s[:fm_end]
                        body = s[fm_end:]
                        new_body = escape_braces_in_math(body)
                        new = front + new_body
                    else:
                        new = escape_braces_in_math(s)
                else:
                    new = escape_braces_in_math(s)

                if new != s:
                    path.write_text(new, encoding='utf-8')
                    return True
                return False


            def main():
                docs_dir = Path('frontend/docs')
                if not docs_dir.exists():
                    print(f'Error: {docs_dir} not found')
                    return

                files = sorted(list(docs_dir.rglob('*.md')) + list(docs_dir.rglob('*.mdx')))
                print(f'Found {len(files)} markdown files')
                fixed = 0
                for f in files:
                    try:
                        if fix_file(f):
                            rel = f.relative_to(docs_dir.parent)
                            print(f'✓ Fixed: {rel}')
                            fixed += 1
                    except Exception as e:
                        print(f'✗ Error {f}: {e}')

                print(f'\nTotal files modified: {fixed}')


            if __name__ == '__main__':
                main()
