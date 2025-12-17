# 🏗️ Backend Architecture & Data Flow

## System Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                         Your Machine                               │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  ┌─────────────────────────────────────┐                          │
│  │   Frontend (React/Docusaurus)       │                          │
│  │   http://localhost:3000             │                          │
│  ├─────────────────────────────────────┤                          │
│  │ - Homepage with CTAs                │                          │
│  │ - Chatbot sidebar                   │                          │
│  │ - Translation button                │                          │
│  │ - Calls /chat, /translate endpoints │                          │
│  └────────────────┬────────────────────┘                          │
│                   │                                                │
│                   │ HTTP (localhost:8000)                          │
│                   ▼                                                │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │   Backend (FastAPI)                                         │  │
│  │   http://localhost:8000                                     │  │
│  ├─────────────────────────────────────────────────────────────┤  │
│  │                                                             │  │
│  │  ┌────────────────────────────────────────────────────┐    │  │
│  │  │ API Routers                                        │    │  │
│  │  ├────────────────────────────────────────────────────┤    │  │
│  │  │ • POST /chat       → rag_service.ask_question()   │    │  │
│  │  │ • POST /translate  → Gemini translation           │    │  │
│  │  │ • GET /health      → Health check                 │    │  │
│  │  └────────────────────────────────────────────────────┘    │  │
│  │                        ▲                                    │  │
│  │                        │                                    │  │
│  │  ┌────────────────────┴─────────────────────────────────┐  │  │
│  │  │ RAG Service (rag_service.py)                        │  │  │
│  │  ├─────────────────────────────────────────────────────┤  │  │
│  │  │ initialize_rag_pipeline():                          │  │  │
│  │  │  1. Load markdown from frontend/docs/book           │  │  │
│  │  │  2. Compute embeddings (sentence-transformers)      │  │  │
│  │  │  3. Recreate Qdrant collection                      │  │  │
│  │  │  4. Upsert vectors + metadata                       │  │  │
│  │  │                                                     │  │  │
│  │  │ ask_question(query):                               │  │  │
│  │  │  1. Embed query                                    │  │  │
│  │  │  2. Search Qdrant for similar docs                 │  │  │
│  │  │  3. Send context + query to Gemini                 │  │  │
│  │  │  4. Return answer + sources                        │  │  │
│  │  │                                                     │  │  │
│  │  │ LLM (Gemini if configured, else Dummy)             │  │  │
│  │  └─────┬──────────────────┬──────────────────────────┘  │  │
│  │        │                  │                              │  │
│  │        ▼                  ▼                              │  │
│  │  ┌──────────────┐  ┌────────────────────────────────┐   │  │
│  │  │ Embedding    │  │ Vector Store                   │   │  │
│  │  │ Generator    │  │ (vector_store.py)              │   │  │
│  │  ├──────────────┤  ├────────────────────────────────┤   │  │
│  │  │ Model:       │  │ Connects to Qdrant Cloud:      │   │  │
│  │  │ all-MiniLM-  │  │ - Read QDRANT_URL env var     │   │  │
│  │  │ L6-v2        │  │ - Read QDRANT_API_KEY env var │   │  │
│  │  │ (384 dims)   │  │ - Fallback: localhost:6333     │   │  │
│  │  │              │  │ - Recreate collection          │   │  │
│  │  │ Transforms   │  │ - Upsert documents             │   │  │
│  │  │ text → vec   │  │ - Search queries               │   │  │
│  │  └──────────────┘  └────────┬─────────────────────────┘   │  │
│  │                             │                              │  │
│  └─────────────────────────────┼──────────────────────────────┘  │
│                                │                                 │
└────────────────────────────────┼─────────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
        ┌───────────▼──────────┐  ┌──────────▼──────────┐
        │  Gemini API          │  │  Qdrant Cloud       │
        │  (Google)            │  │  (Vector DB)        │
        ├──────────────────────┤  ├─────────────────────┤
        │ HTTPS API            │  │ https://cluster-    │
        │ API key auth         │  │ name.qdrant.io      │
        │ Returns text         │  │ API key auth        │
        │ Max free: unlimited  │  │ Free: 1GB + 100 req │
        └──────────────────────┘  └─────────────────────┘
```

## Data Flow: Chat Request

```
User Types Question
       │
       ▼
POST /chat (query, user_level)
       │
       ▼
RAGService.ask_question()
       │
       ├─→ EmbeddingGenerator.embed(query)
       │   └─→ sentence-transformers model
       │       Returns 384-dim vector
       │
       ├─→ VectorStore.search(query_vector)
       │   └─→ Connect to Qdrant Cloud (QDRANT_URL + QDRANT_API_KEY)
       │       Returns top-3 similar docs with metadata
       │
       ├─→ Prepare context from retrieved docs
       │   └─→ Combine title + text snippets
       │
       ├─→ GeminiLLM.invoke(prompt)
       │   └─→ HTTP POST to Gemini API with GEMINI_API_KEY
       │       Returns answer text
       │
       └─→ Return {response, source_documents}
           │
           ▼
        Frontend receives
           │
           ▼
        Display to user
```

## Data Flow: Translation Request

```
User Clicks "Translate to Urdu"
       │
       ▼
POST /translate (text, target_language)
       │
       ▼
GeminiLLM.invoke(prompt)
       │
       └─→ HTTP POST to Gemini API
           "Translate to {language}: {text}"
           │
           ▼
        Gemini returns translated text
           │
           ▼
        Frontend receives
           │
           ▼
        Display translated text / Replace page content
```

## File Organization

```
Backend Components:
├── backend/src/
│   ├── main.py
│   │   ├─ Loads .env using python-dotenv
│   │   ├─ Creates FastAPI app
│   │   ├─ Registers routers (chat, translate, profile)
│   │   └─ Startup event → initializes RAG pipeline
│   │
│   ├── services/
│   │   ├─ rag_service.py
│   │   │   ├─ RAGService class
│   │   │   ├─ initialize_rag_pipeline() → load docs, embed, upsert
│   │   │   ├─ ask_question() → retrieve, format, LLM call
│   │   │   ├─ GeminiLLM class (uses google.generativeai)
│   │   │   └─ DummyLLM class (fallback)
│   │   │
│   │   ├─ vector_store.py
│   │   │   ├─ VectorStore class
│   │   │   ├─ __init__() → reads QDRANT_URL/API_KEY env vars
│   │   │   ├─ recreate_collection()
│   │   │   ├─ upsert_documents() with embeddings
│   │   │   └─ search() for vector similarity
│   │   │
│   │   ├─ embedding_generator.py
│   │   │   ├─ EmbeddingGenerator class
│   │   │   ├─ Uses sentence-transformers (all-MiniLM-L6-v2)
│   │   │   └─ generate_embeddings(texts) → list of vectors
│   │   │
│   │   ├─ content_loader.py
│   │   │   ├─ ContentLoader class
│   │   │   ├─ Loads from frontend/docs/book
│   │   │   └─ Parses frontmatter (id, title)
│   │   │
│   │   ├─ translate.py (not in services, but api/)
│   │   │   └─ Uses Gemini for translation
│   │   │
│   │   └─ ... (other services)
│   │
│   └── api/
│       ├─ chat.py
│       │   ├─ rag_service global instance
│       │   ├─ POST /chat → ask_question()
│       │   └─ POST /ask → ask_question()
│       │
│       └─ translate.py
│           └─ POST /translate → Gemini translation
│
├── scripts/
│   ├─ populate_qdrant.py
│   │   ├─ One-time: load markdown → embeddings → Qdrant
│   │   └─ Uses ContentLoader, EmbeddingGenerator, VectorStore
│   │
│   └─ validate_env.py
│       └─ Check .env before startup
│
└── requirements.txt
    ├─ fastapi, uvicorn
    ├─ qdrant-client
    ├─ sentence-transformers (embeddings)
    ├─ langchain (for RAG)
    ├─ google-generativeai (Gemini LLM)
    └─ python-dotenv (load .env)

Configuration:
├─ .env.example (template)
├─ .env (YOUR CREDENTIALS - you create)
│   ├─ GEMINI_API_KEY
│   ├─ GEMINI_MODEL
│   ├─ QDRANT_URL
│   └─ QDRANT_API_KEY

Setup:
├─ setup_backend.bat (Windows batch)
├─ setup_backend.ps1 (Windows PowerShell)
└─ (or manual steps in README_BACKEND.md)

Documentation:
├─ START_HERE.md
├─ QUICK_REFERENCE.txt
├─ README_BACKEND.md
├─ SETUP_CHECKLIST.md
├─ BACKEND_GEMINI_QDRANT.md
├─ SETUP_BACKEND.md
├─ FILE_INVENTORY.md
├─ STATUS.txt
└─ COMPLETION_SUMMARY.md (this file)
```

## Environment Variable Flow

```
.env file (on your machine)
    │
    ├─ GEMINI_API_KEY
    │   └─→ backend/src/services/rag_service.py
    │       └─→ GeminiLLM(api_key=gemini_api_key)
    │           └─→ genai.configure(api_key=...)
    │               └─→ Gemini API calls
    │
    ├─ GEMINI_MODEL (default: "gemini-pro")
    │   └─→ GeminiLLM(model=...)
    │
    ├─ QDRANT_URL (https://cluster.qdrant.io:6333)
    │   └─→ backend/src/services/vector_store.py
    │       └─→ QdrantClient(url=url, api_key=api_key)
    │
    └─ QDRANT_API_KEY
        └─→ QdrantClient(url=..., api_key=api_key)
            └─→ Qdrant Cloud authentication
```

## Initialization Sequence

```
1. Application Startup
   └─→ backend/src/main.py loads

2. load_dotenv()
   └─→ Reads .env file into environment variables

3. FastAPI App Creation
   └─→ Creates app instance
   └─→ Registers routers (chat, translate, profile)

4. @app.on_event("startup")
   └─→ Runs startup_event()
       └─→ Calls chat.rag_service.initialize_rag_pipeline()
           │
           ├─→ ContentLoader.load_markdown_content()
           │   └─→ Reads frontend/docs/book/*.md
           │
           ├─→ EmbeddingGenerator.generate_embeddings(texts)
           │   └─→ Downloads sentence-transformers model (first run)
           │   └─→ Computes 384-dim vectors for each text
           │
           ├─→ VectorStore.recreate_collection()
           │   └─→ Connects to Qdrant Cloud (uses QDRANT_URL + QDRANT_API_KEY)
           │   └─→ Creates/recreates "textbook_chapters" collection
           │
           └─→ VectorStore.upsert_documents(docs, embeddings)
               └─→ Uploads vectors + metadata to Qdrant Cloud
               └─→ Collection ready for search

5. Backend Ready
   └─→ Listens on http://0.0.0.0:8000
   └─→ Ready to receive /chat and /translate requests
```

## Request Handling

```
Frontend Request (Chat)
    │
    └─→ POST http://localhost:8000/chat
        {query: "...", user_level: "Beginner"}
        │
        └─→ backend/src/api/chat.py:chat_endpoint()
            │
            ├─→ Validate request
            │
            └─→ rag_service.ask_question(query, user_level)
                │
                ├─→ Embed query (384 dims)
                │
                ├─→ Qdrant.search(query_vector, limit=3)
                │   └─→ HTTPS to Qdrant Cloud
                │   └─→ Vector similarity search
                │   └─→ Returns top 3 docs with scores
                │
                ├─→ Format context from retrieved docs
                │
                ├─→ Create prompt:
                │   "Context: {retrieved_docs}
                │    Question: {query}
                │    Adapt for {user_level} level
                │    Answer:"
                │
                ├─→ GeminiLLM.invoke(prompt)
                │   └─→ HTTPS POST to Gemini API
                │   └─→ With GEMINI_API_KEY auth
                │   └─→ Returns answer text
                │
                └─→ Return {response: answer, source_documents: sources}
                    │
                    └─→ Frontend receives JSON
                        └─→ Displays to user
```

## Key Technologies

```
Embeddings:           sentence-transformers (all-MiniLM-L6-v2)
Vector Database:      Qdrant Cloud (hosted SaaS)
LLM:                  Google Gemini API
Backend Framework:    FastAPI (Python)
Vector Size:          384 dimensions
Collection Name:      textbook_chapters
Free Tier Limits:     Qdrant: 1GB + 100 requests/month
                      Gemini: Unlimited (reasonable use)
Content Source:       frontend/docs/book (markdown files)
```

---

This architecture is **scalable, maintainable, and fully functional** with free tiers!
