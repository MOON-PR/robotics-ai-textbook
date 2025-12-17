import os
from typing import List, Tuple

from backend.src.services.content_loader import ContentLoader
from backend.src.services.embedding_generator import EmbeddingGenerator
from backend.src.services.vector_store import VectorStore


class DummyLLM:
    def __init__(self):
        pass

    def invoke(self, prompt: str) -> str:
        # Return varied sample AI responses for demo purposes
        prompt_lower = prompt.lower()
        if "robot" in prompt_lower:
            return "A robot is a machine that can carry out a complex series of actions automatically, especially one programmable by a computer. In robotics, robots are designed to perform tasks that are dangerous, repetitive, or require precision."
        elif "electronics" in prompt_lower:
            return "Electronics is the branch of physics and technology concerned with the design of circuits using transistors and microchips. In robotics, electronics are used to control motors, sensors, and communication systems."
        elif "programming" in prompt_lower or "code" in prompt_lower:
            return "Programming in robotics involves writing software to control robot behavior. Common languages include Python, C++, and ROS (Robot Operating System). It includes algorithms for motion planning, sensor processing, and decision making."
        elif "sensors" in prompt_lower:
            return "Sensors in robotics detect environmental changes and provide data to the robot's control system. Common types include ultrasonic sensors for distance, infrared for proximity, cameras for vision, and IMU for orientation."
        elif "motors" in prompt_lower or "actuators" in prompt_lower:
            return "Motors and actuators convert electrical energy into mechanical motion. In robotics, servo motors provide precise control, DC motors offer speed, and stepper motors enable accurate positioning."
        else:
            return "This is a sample AI response about robotics. The system provides information on topics like robots, electronics, programming, sensors, and motors. Please ask a specific question for more detailed information."


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
        # Use the current Gemini API
        try:
            model = self.genai.GenerativeModel(self.model)
            response = model.generate_content(prompt)
            return response.text
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
        print(f"Gemini API key present: {bool(gemini_api_key)}")
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

    def initialize_demo_mode(self) -> bool:
        """Enable demo mode: mark pipeline as initialized but skip actual embedding/vector operations.
        
        This allows the chat endpoint to work immediately with canned responses for demos/submissions.
        Returns True always.
        """
        self._initialized = True
        print("Demo mode enabled: RAG pipeline marked as initialized (using canned responses).")
        return True

    def _retrieve_and_format(self, query: str):
        # Generate embedding for the query
        query_embedding = self.embedding_generator.generate_embeddings([query])[0]

        # Search the vector store
        search_results = self.vector_store.search(query_embedding, limit=1)  # Retrieve top 1

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

        return {"context_str": "\n\n".join(context_for_llm)[:2000], "sources": source_documents}

    def ask_question(self, query: str, user_level: str = "Beginner") -> Tuple[str, List[dict]]:
        if not self._initialized:
            print("RAG pipeline not initialized. Using LLM without context.")
            # Use LLM directly without RAG
            adapted_template = f"Please answer this question about robotics: {query}"
            if user_level == "Beginner":
                adapted_template += "\nProvide a simplified explanation."
            elif user_level == "Intermediate":
                adapted_template += "\nInclude practical examples if relevant."
            elif user_level == "Advanced":
                adapted_template += "\nFocus on underlying algorithms or mathematical principles if applicable."
            response_text = self.llm.invoke(adapted_template)
            print(f"LLM Response: {response_text}")
            return response_text, []

        # In demo mode, skip retrieval and return canned response
        if os.getenv("DEMO_MODE", "false").lower() in ("true", "1", "yes"):
            response_text = self.llm.invoke(query)  # Direct query to LLM without context
            return response_text, []

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