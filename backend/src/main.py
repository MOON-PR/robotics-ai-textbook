import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
from backend.src.api import chat, profile, translate  # Import routers

# Load environment variables from .env file
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '..', '.env'))
print(f"GEMINI_API_KEY after load_dotenv: {os.environ.get('GEMINI_API_KEY')}")

app = FastAPI()

# Enable CORS for local development so the frontend (localhost:3000)
# can call the backend API from the browser.
app.add_middleware(
    CORSMiddleware,
    # Allow all origins during local development to avoid CORS issues.
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat.router)
app.include_router(profile.router)
app.include_router(translate.router)


@app.on_event("startup")
def startup_event():
    # Initialize RAG pipeline (populate Qdrant) at app startup
    try:
        demo_mode = os.getenv("DEMO_MODE", "false").lower() in ("true", "1", "yes")
        if demo_mode:
            print("Initializing RAG pipeline in demo mode...")
            chat.rag_service.initialize_demo_mode()
        else:
            print("Initializing RAG pipeline...")
            chat.rag_service.initialize_rag_pipeline()
    except Exception as e:
        print(f"RAG pipeline initialization failed at startup: {e}")
        # Do not crash the app, just log the error
        pass


@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/health")
def health_check():
    return {"status": "ok"}


@app.get("/health")
def health_check():
    return {"status": "ok"}
