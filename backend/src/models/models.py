from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    query: str
    user_level: Optional[str] = "Beginner" # Default for personalization

class ChatResponse(BaseModel):
    response: str
    source_documents: Optional[list[dict]] = None # To include source documents later
