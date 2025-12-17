"""Update internal links that point to old flat `/docs/NN-section/...` routes
and convert them to the generated folder-based routes used by Docusaurus.

This script edits files under `frontend` (md, mdx, tsx, jsx, html, json)
and writes changes in place, reporting modified files.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDE_DIRS = {"build", "node_modules", ".git"}
EXTS = {".md", ".mdx", ".tsx", ".ts", ".jsx", ".js", ".html", ".json"}

# Mapping: numeric prefix -> folder name
MAPPINGS = {
    "01": "introduction",
    "02": "electronics",
    "03": "programming",
    "04": "sensors",
    "05": "microcontrollers",
    "06": "algorithms",
    "07": "computer-vision",
    "08": "ai-robotics",
    "09": "projects",
    "10": "hackathon-q4",
    "11": "appendix",
}

# Build regex replacements: for each number, turn /docs/NN-section/(rest) into /docs/<folder>/book-NN-section<rest>
REPLACEMENTS = []
for num, folder in MAPPINGS.items():
    # match paths like /docs/01-introduction/... or /docs/01-introduction
    pattern = re.compile(rf"(/docs/{num}-[a-z0-9-]+)(/[^)\"'\s]*)?", flags=re.IGNORECASE)
    repl = lambda m, n=num, f=folder: f"/docs/{f}/book-{n}-{m.group(1).split('/',2)[-1]}" + (m.group(2) or "")
    REPLACEMENTS.append((pattern, repl))


def should_skip(path: Path):
    for part in path.parts:
        if part in EXCLUDE_DIRS:
            return True
    return False


def process_file(path: Path):
    text = path.read_text(encoding='utf-8')
    new_text = text
    changed = False
    for pattern, repl in REPLACEMENTS:
        # Use a function so we can reconstruct new id correctly
        def _sub(m):
            try:
                return repl(m)
            except Exception:
                return m.group(0)
        new_text = pattern.sub(_sub, new_text)
    if new_text != text:
        path.write_text(new_text, encoding='utf-8')
        changed = True
    return changed


def main():
    changed_files = []
    for p in ROOT.rglob("*"):
        if p.is_dir():
            continue
        if should_skip(p):
            continue
        if p.suffix.lower() not in EXTS:
            continue
        try:
            if process_file(p):
                changed_files.append(str(p.relative_to(ROOT)))
        except Exception as e:
            print(f"Failed processing {p}: {e}")
    print(f"Updated {len(changed_files)} files:")
    for f in changed_files:
        print(f" - {f}")


if __name__ == '__main__':
    main()
