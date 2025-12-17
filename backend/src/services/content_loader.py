import os
import re

class ContentLoader:
    def __init__(self, chapters_dir="frontend/docs/book"):
        # The book markdown files are located under frontend/docs/book
        # Update this if you change where the content lives.
        self.chapters_dir = chapters_dir

    def load_markdown_content(self):
        documents = []
        for root, _, files in os.walk(self.chapters_dir):
            for file_name in files:
                if file_name.endswith(".md"):
                    file_path = os.path.join(root, file_name)
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = f.read()
                        
                        # Extract metadata (id, title) from frontmatter
                        metadata = {}
                        match = re.match(r"^\s*---\s*\n(.*?)\n---\s*\n(.*)", content, re.DOTALL)
                        if match:
                            frontmatter = match.group(1)
                            body = match.group(2)
                            
                            id_match = re.search(r"id:\s*(.*)", frontmatter)
                            if id_match:
                                metadata["id"] = id_match.group(1).strip()
                            
                            title_match = re.search(r"title:\s*(.*)", frontmatter)
                            if title_match:
                                metadata["title"] = title_match.group(1).strip()
                            
                            documents.append({"text": body.strip(), "metadata": metadata})
                        else:
                            documents.append({"text": content.strip(), "metadata": {"source": file_name}})
        return documents

if __name__ == "__main__":
    loader = ContentLoader()
    documents = loader.load_markdown_content()
    for doc in documents:
        print(f"Metadata: {doc['metadata']}")
        print(f"Content snippet: {doc['text'][:200]}...")
        print("-" * 50)