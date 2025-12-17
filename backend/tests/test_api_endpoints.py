from fastapi.testclient import TestClient
from backend.src.main import app
from unittest.mock import patch, MagicMock
import pytest

client = TestClient(app)

@pytest.fixture(autouse=True)
def mock_rag_service_initialization():
    # Mock the RAGService initialization during app startup
    with patch('backend.src.api.chat.rag_service.initialize_rag_pipeline') as mock_init:
        mock_init.return_value = True # Assume successful initialization
        yield

@pytest.fixture
def mock_rag_ask_question():
    with patch('backend.src.api.chat.rag_service.ask_question') as mock_ask:
        yield mock_ask

def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}

def test_chat_endpoint(mock_rag_ask_question):
    mock_rag_ask_question.return_value = ("Mocked RAG response.", [{"id": "doc1", "title": "Doc Title"}])
    response = client.post("/chat", json={"query": "test query", "user_level": "Beginner"})
    assert response.status_code == 200
    assert response.json() == {"response": "Mocked RAG response.", "source_documents": [{"id": "doc1", "title": "Doc Title"}]}
    mock_rag_ask_question.assert_called_once_with("test query", "Beginner")

def test_ask_endpoint(mock_rag_ask_question):
    mock_rag_ask_question.return_value = ("Mocked RAG answer.", [])
    response = client.post("/ask", json={"query": "test question", "user_level": "Advanced"})
    assert response.status_code == 200
    assert response.json() == {"response": "Mocked RAG answer.", "source_documents": []}
    mock_rag_ask_question.assert_called_once_with("test question", "Advanced")

def test_get_user_profile():
    # Test with a new user
    response = client.get("/profile/testuser1")
    assert response.status_code == 200
    assert response.json() == {"username": "testuser1", "level": "Beginner"}

    # Test updating a user profile
    client.post("/profile", json={"username": "testuser2", "level": "Intermediate"})
    response = client.get("/profile/testuser2")
    assert response.status_code == 200
    assert response.json() == {"username": "testuser2", "level": "Intermediate"}

def test_update_user_profile():
    response = client.post("/profile", json={"username": "testuser3", "level": "Advanced"})
    assert response.status_code == 200
    assert response.json() == {"username": "testuser3", "level": "Advanced"}

    # Verify it's updated
    response = client.get("/profile/testuser3")
    assert response.status_code == 200
    assert response.json() == {"username": "testuser3", "level": "Advanced"}

def test_translate_endpoint():
    response = client.post("/translate", json={"text": "Hello", "target_language": "ur"})
    assert response.status_code == 200
    assert "Urdu translation of 'Hello': [Placeholder for actual Urdu translation]" in response.json()["translated_text"]
    assert response.json()["original_text"] == "Hello"

