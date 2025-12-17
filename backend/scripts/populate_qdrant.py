"""Populate Qdrant collection from markdown files under frontend/docs/book.

Usage:
  python backend/scripts/populate_qdrant.py

This script uses the existing services in `backend/src/services` to load content,
compute embeddings, recreate the Qdrant collection, and upsert documents.

Make sure Qdrant is running (docker compose up -d qdrant) and your virtualenv
is active with dependencies installed.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# Add repository root to sys.path so `import backend...` works when
# running this script directly (e.g. `python backend/scripts/populate_qdrant.py`).
sys.path.insert(0, str(ROOT))

from backend.src.services.content_loader import ContentLoader
from backend.src.services.embedding_generator import EmbeddingGenerator
from backend.src.services.vector_store import VectorStore


def main():
    print("Loading markdown documents from frontend/docs/book...")
    loader = ContentLoader()
    documents = loader.load_markdown_content()
    if not documents:
        print("No documents found. Ensure frontend/docs/book contains markdown files.")
        return

    texts = [doc.get("text", "") for doc in documents]
    print(f"Found {len(texts)} documents. Generating embeddings (this may take time)...")

    emb_gen = EmbeddingGenerator()
    embeddings = emb_gen.generate_embeddings(texts)

    print("Preparing metadata and upserting into Qdrant...")
    prepared_docs = []
    for i, doc in enumerate(documents):
        metadata = doc.get("metadata", {})
        metadata.setdefault("id", f"doc_{i}")
        metadata.setdefault("title", metadata.get("id"))
        metadata["text"] = texts[i][:500]
        prepared_docs.append({"metadata": metadata})

    store = VectorStore()
    try:
        store.recreate_collection()
    except Exception as e:
        print(f"Warning: could not recreate collection: {e}")

    store.upsert_documents(prepared_docs, embeddings)
    print("Upsert complete. You can now query the backend chat endpoint.")


if __name__ == '__main__':
    main()
