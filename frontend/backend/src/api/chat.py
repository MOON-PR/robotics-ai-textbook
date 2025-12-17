from fastapi import APIRouter
from backend.src.models.models import ChatRequest, ChatResponse
from backend.src.services.rag_service import RAGService

router = APIRouter()

# Global instance of RAGService
rag_service = RAGService()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Endpoint to handle chat requests and provide RAG-based responses.
    """
    print("CHAT ENDPOINT HIT")
    # Temporary test response
    return ChatResponse(response="BACKEND HIT CONFIRMED - NO DEMO", source_documents=[])

@router.post("/ask", response_model=ChatResponse)
async def ask_endpoint(request: ChatRequest):
    """
    Endpoint to handle direct question answering from the RAG service.
    """
    response_text, source_documents = rag_service.ask_question(request.query, request.user_level)
    return ChatResponse(response=response_text, source_documents=source_documents)
