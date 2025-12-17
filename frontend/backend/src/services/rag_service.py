import os
from typing import List, Tuple

from backend.src.services.content_loader import ContentLoader
from backend.src.services.embedding_generator import EmbeddingGenerator
from backend.src.services.vector_store import VectorStore


class DummyLLM:
    def __init__(self):
        pass

    def invoke(self, prompt: str) -> str:
        # Friendly placeholder response when no real LLM is configured
        return "[LLM not configured] This is a placeholder response. Configure GEMINI_API_KEY or OLLAMA_MODEL to use a real model."


class GeminiLLM:
    def __init__(self, model: str = "gemini-pro", api_key: str | None = None):
        try:
            import google.generativeai as genai  # type: ignore
        except Exception as e:
            raise RuntimeError(
                "google.generativeai package is not installed or failed to import. Install it to use Gemini integration."
            ) from e
        self.genai = genai
        if api_key:
            # configure global api key
            try:
                self.genai.configure(api_key=api_key)
            except Exception:
                # some versions use a different configure method; ignore if it fails
                pass
        self.model = model

    def invoke(self, prompt: str) -> str:
        # Use the chat API if available
        try:
            # Attempt chat-style call
            resp = self.genai.chat.create(model=self.model, messages=[{"role": "user", "content": prompt}])
            # Gen AI client returns different shapes; attempt to extract text
            if hasattr(resp, "candidates") and resp.candidates:
                return resp.candidates[0].content
            if hasattr(resp, "response"):
                return getattr(resp, "response")
            return str(resp)
        except Exception as e:
            return f"[Gemini error] {e}"


class RAGService:
    def __init__(self, collection_name: str = "textbook_chapters"):
        self.content_loader = ContentLoader()
        self.embedding_generator = EmbeddingGenerator()
        self.vector_store = VectorStore(collection_name=collection_name)

        # select LLM based on environment variables
        self.llm = self._initialize_llm()

        self.rag_template = (
            "You are an assistant for question-answering about the provided textbook content. "
            "Use the following retrieved context to answer the question. If the question cannot be answered from the provided context, "
            'state "I cannot answer this question based on the provided textbook content."\n\n'
            "Context: {context}\n"
            "Question: {question}\n"
            "Answer:"
        )

        # state for whether the pipeline has been initialized (Qdrant populated)
        self._initialized = False

    def _initialize_llm(self):
        # Prioritize Gemini if configured, else fallback to DummyLLM
        gemini_api_key = os.environ.get("GEMINI_API_KEY")
        gemini_model = os.environ.get("GEMINI_MODEL", "gemini-pro")
        if gemini_api_key:
            try:
                return GeminiLLM(model=gemini_model, api_key=gemini_api_key)
            except Exception as e:
                print(f"Could not initialize Gemini LLM: {e}. Falling back to dummy LLM.")

        # Optionally, support Ollama or other local LLMs here in future
        # If not configured, return DummyLLM so service remains functional
        return DummyLLM()

    def initialize_rag_pipeline(self) -> bool:
        """Load markdown content, create embeddings, recreate Qdrant collection and upsert points.

        Returns True on success, False on failure.
        """
        try:
            documents = self.content_loader.load_markdown_content()
            if not documents:
                print("No documents found by ContentLoader. Check chapters_dir path.")
                return False

            texts = [doc.get("text", "") for doc in documents]

            # generate embeddings in batches
            embeddings = self.embedding_generator.generate_embeddings(texts)

            # ensure metadata contains id/title and include full content for retrieval if needed
            prepared_docs = []
            for i, doc in enumerate(documents):
                metadata = doc.get("metadata", {})
                # include content snippet in payload for easier debugging
                metadata["text"] = texts[i][:500]
                # ensure id/title exist
                metadata.setdefault("id", f"doc_{i}")
                metadata.setdefault("title", metadata.get("id"))
                prepared_docs.append({"metadata": metadata})

            # recreate Qdrant collection and upsert
            try:
                self.vector_store.recreate_collection()
            except Exception as e:
                print(f"Warning: could not recreate collection: {e}")

            self.vector_store.upsert_documents(prepared_docs, embeddings)

            self._initialized = True
            print("RAG pipeline initialized: Qdrant collection populated.")
            return True
        except Exception as e:
            print(f"Failed to initialize RAG pipeline: {e}")
            return False

    def _retrieve_and_format(self, query: str):
        # Generate embedding for the query
        query_embedding = self.embedding_generator.generate_embeddings([query])[0]

        # Search the vector store
        search_results = self.vector_store.search(query_embedding, limit=3)  # Retrieve top 3

        context_for_llm = []
        source_documents = []
        for res in search_results:
            payload = res.get("metadata", {})
            source_documents.append({
                "id": payload.get("id"),
                "title": payload.get("title"),
                "score": res.get("score"),
            })
            context_for_llm.append(f"Title: {payload.get('title', payload.get('id'))}\n{payload.get('text','')}")

        return {"context_str": "\n\n".join(context_for_llm), "sources": source_documents}

    def ask_question(self, query: str, user_level: str = "Beginner") -> Tuple[str, List[dict]]:
        if not self._initialized:
            print("RAG pipeline not initialized. Please call initialize_rag_pipeline() first.")
            return "Error: RAG pipeline not initialized.", []

        retrieval_data = self._retrieve_and_format(query)
        context = retrieval_data["context_str"]
        sources = retrieval_data["sources"]

        # Adapt prompt based on user_level
        adapted_template = self.rag_template
        if user_level == "Beginner":
            adapted_template += "\nProvide a simplified explanation."
        elif user_level == "Intermediate":
            adapted_template += "\nInclude practical examples if relevant."
        elif user_level == "Advanced":
            adapted_template += "\nFocus on underlying algorithms or mathematical principles if applicable."

        # Format prompt using simple string formatting (no langchain dependency)
        formatted_prompt = adapted_template.format(
            context=context,
            question=query,
        )

        # invoke the configured LLM
        response_text = self.llm.invoke(formatted_prompt)
        return response_text, sources


if __name__ == "__main__":
    # Example quick-run for debugging (requires Qdrant running)
    rag_service = RAGService()
    ok = rag_service.initialize_rag_pipeline()
    print(f"Initialized: {ok}")
    if ok:
        answer_text, sources = rag_service.ask_question("What is Physical AI?")
        print(f"Answer: {answer_text}")
        print(f"Sources: {sources}")