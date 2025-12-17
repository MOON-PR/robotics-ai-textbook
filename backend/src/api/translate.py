from fastapi import APIRouter
from pydantic import BaseModel
from typing import Optional
import os

router = APIRouter()


class TranslateRequest(BaseModel):
    text: str
    target_language: str = "ur"  # Urdu


class TranslateResponse(BaseModel):
    original_text: str
    translated_text: str


@router.post("/translate", response_model=TranslateResponse)
async def translate_text(request: TranslateRequest):
    """
    Translate text to the target language. If GEMINI_API_KEY is provided
    the endpoint will attempt to use Gemini (Google Generative AI) to
    perform the translation. Otherwise it returns a placeholder message.
    """
    gemini_api_key = os.environ.get("GEMINI_API_KEY")
    gemini_model = os.environ.get("GEMINI_MODEL", "gemini-pro")

    if gemini_api_key:
        try:
            import google.generativeai as genai  # type: ignore
            genai.configure(api_key=gemini_api_key)
            prompt = (
                f"Translate the following text to {request.target_language} preserving code blocks and formatting.\n\n" + request.text
            )
            resp = genai.chat.create(model=gemini_model, messages=[{"role": "user", "content": prompt}])
            # Extract response text
            translated = None
            if hasattr(resp, "candidates") and resp.candidates:
                translated = resp.candidates[0].content
            elif hasattr(resp, "response"):
                translated = getattr(resp, "response")
            else:
                translated = str(resp)

            return TranslateResponse(original_text=request.text, translated_text=translated)
        except Exception as e:
            # Fall back to placeholder response on failure
            placeholder = f"[Translation failed: {e}] Urdu translation of the provided text is not available."
            return TranslateResponse(original_text=request.text, translated_text=placeholder)

    # Default placeholder behavior when no translation provider configured
    translated_text = f"Urdu translation of the provided text: [Translation provider not configured]"
    return TranslateResponse(original_text=request.text, translated_text=translated_text)
