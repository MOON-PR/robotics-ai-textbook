import pytest
from backend.src.services.rag_service import RAGService
from backend.src.services.content_loader import ContentLoader
from unittest.mock import MagicMock, patch

@pytest.fixture
def rag_service_instance():
    # Mock dependencies for isolated testing
    with patch('backend.src.services.content_loader.ContentLoader') as MockContentLoader, \
         patch('backend.src.services.embedding_generator.EmbeddingGenerator') as MockEmbeddingGenerator, \
         patch('backend.src.services.vector_store.VectorStore') as MockVectorStore:
        
        # Configure mocks
        mock_content_loader_instance = MockContentLoader.return_value
        mock_content_loader_instance.load_markdown_content.return_value = [
            {"text": "Physical AI is a field combining AI and robotics.", "metadata": {"id": "intro", "title": "Intro"}},
            {"text": "Humanoid robots mimic human form.", "metadata": {"id": "humanoids", "title": "Humanoids"}},
        ]

        mock_embedding_generator_instance = MockEmbeddingGenerator.return_value
        mock_embedding_generator_instance.generate_embeddings.side_effect = lambda texts: [[0.1]*384 for _ in texts]

        mock_vector_store_instance = MockVectorStore.return_value
        mock_vector_store_instance.search.return_value = [
            MagicMock(payload={"id": "intro", "title": "Intro"}, score=0.9),
        ]
        
        # Initialize RAGService with mocked dependencies
        service = RAGService()
        service.initialize_rag_pipeline()
        return service

def test_rag_service_initialization(rag_service_instance):
    assert rag_service_instance.rag_template is not None
    assert rag_service_instance.llm is not None
    rag_service_instance.content_loader.load_markdown_content.assert_called_once()
    rag_service_instance.embedding_generator.generate_embeddings.assert_called_once()
    rag_service_instance.vector_store.recreate_collection.assert_called_once()
    rag_service_instance.vector_store.upsert_documents.assert_called_once()

def test_ask_question_beginner_level(rag_service_instance):
    query = "What is physical AI?"
    response, sources = rag_service_instance.ask_question(query, user_level="Beginner")
    assert "simplified explanation" in response # Placeholder LLM might not be perfect, but should indicate adaptation
    assert len(sources) > 0
    rag_service_instance.embedding_generator.generate_embeddings.assert_called() # Called for query embedding
    rag_service_instance.vector_store.search.assert_called()

def test_ask_question_advanced_level(rag_service_instance):
    query = "Explain the kinematics of humanoid robots."
    response, sources = rag_service_instance.ask_question(query, user_level="Advanced")
    assert "mathematical principles" in response or "algorithms" in response # Placeholder LLM
    assert len(sources) > 0

def test_ask_question_unanswerable(rag_service_instance):
    query = "What color is the sky?"
    response, sources = rag_service_instance.ask_question(query)
    assert "cannot answer this question" in response
    assert len(sources) > 0 # Still retrieves sources, but LLM states it can't answer from context
