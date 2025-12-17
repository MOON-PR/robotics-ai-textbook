#!/usr/bin/env python3
import re
import os
import glob

# Pattern to find escaped underscores outside of math delimiters
# Replace \_ with _ in non-math contexts
def fix_latex_underscores(content):
    # Apply several safe, idempotent regex replacements to normalize math
    s = content

    # 1) Remove outer $ that double-wrap display math like $\[ ... \]$
    s = re.sub(r"\$\s*\\\[", "\\[", s)
    s = re.sub(r"\\\]\s*\$", "\\]", s)

    # 2) Replace occurrences of \_{  and \_word  with _{ and _word
    s = re.sub(r"\\_\{", "_{", s)
    s = re.sub(r"\\_(?=\w)", "_", s)

    # 3) Replace common escaped underscores inside $..$ or $$..$$ (idempotent)
    s = re.sub(r"\$([^$]*?)\$(?=[^$])", lambda m: "$" + m.group(1).replace('\\_', '_') + "$", s)

    # 4) Fix patterns like K\_p, T\_loop, ADC\_{val} etc.
    s = s.replace('K\\_p', 'K_p').replace('K\\_i', 'K_i').replace('K\\_d', 'K_d')
    s = s.replace('T\\_loop', 'T_loop').replace('T\\_F', 'T_F')
    s = s.replace('ADC\\_{val}', 'ADC_{val}').replace('ADC\\_val', 'ADC_val')
    s = s.replace('V\\_in', 'V_in').replace('V\\_{in}', 'V_{in}').replace('V\\_ref', 'V_ref')
    s = s.replace('f\\_PWM', 'f_PWM').replace('f\\_CPU', 'f_CPU')

    return s

# Process all markdown files
book_dir = 'docs/book'
for md_file in glob.glob(os.path.join(book_dir, '**/*.md'), recursive=True):
    with open(md_file, 'r', encoding='utf-8') as f:
        original_content = f.read()
    
    fixed_content = fix_latex_underscores(original_content)
    
    if fixed_content != original_content:
        with open(md_file, 'w', encoding='utf-8') as f:
            f.write(fixed_content)
        print(f"Fixed: {md_file}")
    else:
        print(f"No changes: {md_file}")

print("\nDone!")
