import re
from pathlib import Path

def wrap_math_in_backticks(filepath):
    """Wrap any LaTeX expressions outside backticks in backticks to prevent MDX parsing."""
    p = Path(filepath)
    text = p.read_text(encoding='utf-8')
    
    # Pattern: non-escaped underscore+brace notation (common LaTeX)
    # E.g., R_{total}, hat{x}, etc.
    # We need to wrap these in backticks ONLY if not already inside backticks/code
    
    lines = text.split('\n')
    result = []
    in_fence = False
    
    for line in lines:
        # Detect code fence
        if '```' in line:
            in_fence = not in_fence
            result.append(line)
            continue
        
        if in_fence:
            result.append(line)
            continue
        
        # Outside code fence: wrap LaTeX-like expressions in backticks
        # Pattern: [A-Za-z_]+\{[^}]*\} (word followed by braces)
        # But skip if already inside backticks
        new_line = line
        parts = []
        i = 0
        in_backtick = False
        while i < len(new_line):
            if new_line[i] == '`':
                in_backtick = not in_backtick
                parts.append(new_line[i])
                i += 1
            elif not in_backtick and i < len(new_line) - 2:
                # Check for LaTeX pattern: letters+{...}
                m = re.match(r'([A-Za-z_]+)\{[^}]*\}', new_line[i:])
                if m:
                    matched = m.group(0)
                    # Wrap in backticks if not already wrapped
                    parts.append(f'`{matched}`')
                    i += len(matched)
                else:
                    parts.append(new_line[i])
                    i += 1
            else:
                parts.append(new_line[i])
                i += 1
        
        new_line = ''.join(parts)
        result.append(new_line)
    
    text = '\n'.join(result)
    p.write_text(text, encoding='utf-8')
    print(f'Wrapped LaTeX in backticks: {filepath}')

if __name__ == '__main__':
    wrap_math_in_backticks('frontend/docs/book/04-sensors/04-inertial-measurement-units-and-encoders.md')
