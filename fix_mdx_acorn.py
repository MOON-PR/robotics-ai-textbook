#!/usr/bin/env python3
"""
Fix MDX acorn parsing errors by escaping curly braces and brackets in math expressions.
The issue is that MDX interprets {, }, [, ], $, and other special chars as code/JSX.
We need to escape them so MDX doesn't try to parse them as JavaScript.
"""

import os
import re
from pathlib import Path

def fix_mdx_file(filepath):
    """Fix acorn parsing errors in a single MDX/MD file."""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    original_content = content
    
    # Pattern 1: Fix unescaped curly braces in inline code-like expressions
    # E.g., V = I * R should not be inside {} unless escaped
    # The issue is expressions like {equation} where equation contains { }
    
    # Pattern 2: Fix math expressions - wrap in proper code blocks
    # Replace $ ... $ or $$ ... $$ with proper escaping
    
    # First, let's identify lines with problematic patterns
    lines = content.split('\n')
    fixed_lines = []
    
    for line in lines:
        # Skip code blocks
        if line.strip().startswith('```'):
            fixed_lines.append(line)
            continue
        
        # Skip frontmatter
        if line.strip().startswith('---') or line.strip().startswith('id:') or line.strip().startswith('title:'):
            fixed_lines.append(line)
            continue
        
        # Escape curly braces in math/equation context
        # Pattern: { followed by math operators like +, -, *, /, =, ^
        # This is very conservative - only fix obvious math patterns
        
        # Look for patterns like {something with math operators}
        # and escape the curly braces
        if '{' in line and '}' in line:
            # Check if it looks like a math expression (contains =, +, -, *, /, ^, etc)
            # between the braces
            match = re.search(r'\{([^}]*[=+\-*/^].*?)\}', line)
            if match:
                # This looks like a math expression, escape the braces
                line = re.sub(r'\{([^}]*[=+\-*/^].*?)\}', r'{\\{\1\\}}', line)
        
        # Fix escaped backslash patterns like \[ and \]
        # These should be $$ $$ for block math
        line = re.sub(r'\\?\[([^\]]*)\]', r'$$\1$$', line)
        
        # Fix patterns like \( and \)  
        # These should be $ $ for inline math
        line = re.sub(r'\\?\(([^)]*)\)', r'$\1$', line)
        
        fixed_lines.append(line)
    
    content = '\n'.join(fixed_lines)
    
    # If content changed, write it back
    if content != original_content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        return True
    return False

def main():
    """Fix all MDX/MD files in the docs directory."""
    docs_dir = Path('frontend/docs')
    
    if not docs_dir.exists():
        print(f"Error: {docs_dir} not found")
        return
    
    # Find all .md and .mdx files
    files_to_fix = list(docs_dir.glob('**/*.md')) + list(docs_dir.glob('**/*.mdx'))
    
    print(f"Found {len(files_to_fix)} markdown files to check")
    
    fixed_count = 0
    for filepath in sorted(files_to_fix):
        if fix_mdx_file(str(filepath)):
            print(f"✓ Fixed: {filepath.relative_to(docs_dir.parent)}")
            fixed_count += 1
    
    print(f"\nTotal files fixed: {fixed_count}")

if __name__ == '__main__':
    main()
