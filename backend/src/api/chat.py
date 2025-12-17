from fastapi import APIRouter
from backend.src.models.models import ChatRequest, ChatResponse
from backend.src.services.rag_service import RAGService

router = APIRouter()

# Global instance of RAGService - created lazily
rag_service = None

def get_rag_service():
    global rag_service
    if rag_service is None:
        rag_service = RAGService()
    return rag_service

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Endpoint to handle chat requests and provide RAG-based responses.
    """
    print("CHAT ENDPOINT HIT")
    rag_service = get_rag_service()
    if not rag_service._initialized:
        try:
            print("Initializing RAG pipeline...")
            rag_service.initialize_rag_pipeline()
        except Exception as e:
            print(f"RAG initialization failed: {e}")
            return ChatResponse(response=f"RAG initialization failed: {e}", source_documents=[])
    response_text, source_documents = rag_service.ask_question(request.query, request.user_level)
    return ChatResponse(response=response_text, source_documents=source_documents)

@router.post("/ask", response_model=ChatResponse)
async def ask_endpoint(request: ChatRequest):
    """
    Endpoint to handle direct question answering from the RAG service.
    """
    response_text, source_documents = rag_service.ask_question(request.query, request.user_level)
    return ChatResponse(response=response_text, source_documents=source_documents)
