import os

from qdrant_client import QdrantClient, models

class VectorStore:
    def __init__(self, host: str | None = None, port: int | None = None, url: str | None = None, api_key: str | None = None, collection_name="textbook_chapters"):
        """
        Initialize Qdrant client. Supports:
        - Qdrant Cloud (via url + api_key env vars): QDRANT_URL, QDRANT_API_KEY
        - Localhost (via host/port env vars): QDRANT_HOST, QDRANT_PORT
        """
        # Priority 1: Qdrant Cloud (url + api_key)
        url = url or os.environ.get("QDRANT_URL")
        api_key = api_key or os.environ.get("QDRANT_API_KEY")
        
        if url and api_key:
            print(f"Connecting to Qdrant Cloud at {url}")
            self.client = QdrantClient(url=url, api_key=api_key)
        else:
            # Priority 2: Localhost (Docker or local)
            host = host or os.environ.get("QDRANT_HOST", "localhost")
            port = port or int(os.environ.get("QDRANT_PORT", "6333"))
            print(f"Connecting to Qdrant at {host}:{port}")
            self.client = QdrantClient(host=host, port=port)
        
        self.collection_name = collection_name
        self.vector_size = 384  # Based on 'all-MiniLM-L6-v2' model

    def recreate_collection(self):
        self.client.recreate_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=self.vector_size, distance=models.Distance.COSINE),
        )
        print(f"Collection '{self.collection_name}' recreated.")

    def upsert_documents(self, documents: list[dict], embeddings: list[list[float]]):
        points = []
        for i, doc in enumerate(documents):
            points.append(
                models.PointStruct(
                    id=i,
                    vector=embeddings[i],
                    payload=doc["metadata"],
                )
            )
        
        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )
        print(f"Upserted {len(documents)} documents to collection '{self.collection_name}'.")

    def search(self, query_embedding: list[float], limit: int = 5) -> list[dict]:
        search_result = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=limit,
        )
        
        results = []
        for point in search_result.points:
            results.append({
                "score": point.score,
                "metadata": point.payload,
            })
        return results

if __name__ == "__main__":
    # Example usage:
    # Requires a running Qdrant instance (e.g., via docker-compose)
    
    # 1. Initialize Qdrant client
    vector_store = VectorStore()
    
    # 2. Recreate collection (optional, useful for fresh start)
    try:
        vector_store.recreate_collection()
    except Exception as e:
        print(f"Could not recreate collection (it might not exist yet, or Qdrant is not running): {e}")
        print("Please ensure Qdrant is running, e.g., with 'docker compose up -d qdrant'")
        exit(1)

    # 3. Dummy documents and embeddings (in real scenario, get from content_loader and embedding_generator)
    dummy_documents = [
        {"text": "Introduction to Physical AI concepts.", "metadata": {"id": "intro", "title": "Introduction"}},
        {"text": "Fundamentals of Humanoid Robotics.", "metadata": {"id": "humanoids", "title": "Humanoid Robotics"}},
    ]
    dummy_embeddings = [
        [0.1] * 384,  # Example embedding 1
        [0.2] * 384,  # Example embedding 2
    ]

    # 4. Upsert documents
    vector_store.upsert_documents(dummy_documents, dummy_embeddings)

    # 5. Search for a dummy query
    query_text = "What is physical AI?"
    # In a real app, generate embedding for query_text
    query_embedding = [0.11] * 384 
    
    search_results = vector_store.search(query_embedding)
    print("\nSearch Results:")
    for result in search_results:
        print(result)