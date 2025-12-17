from sentence_transformers import SentenceTransformer

class EmbeddingGenerator:
    def __init__(self, model_name='all-MiniLM-L6-v2'):
        self.model = SentenceTransformer(model_name)

    def generate_embeddings(self, texts: list[str]) -> list[list[float]]:
        embeddings = self.model.encode(texts, convert_to_numpy=False, convert_to_tensor=False)
        return [emb.tolist() for emb in embeddings]

if __name__ == "__main__":
    # Example usage
    generator = EmbeddingGenerator()
    texts = [
        "This is an example sentence.",
        "Each sentence is converted into a vector.",
        "Physical AI and humanoid robotics are fascinating fields."
    ]
    embeddings = generator.generate_embeddings(texts)
    for i, emb in enumerate(embeddings):
        print(f"Text: {texts[i]}")
        print(f"Embedding length: {len(emb)}")
        print(f"Embedding snippet: {emb[:5]}...")
        print("-" * 50)