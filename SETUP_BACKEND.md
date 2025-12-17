# Backend Setup Guide (Gemini + Qdrant Cloud, No Docker)

## Prerequisites

- Python 3.9+ installed
- Qdrant Cloud cluster (free tier available at https://cloud.qdrant.io)
- Google Gemini API key (from https://ai.google.dev)

## Step 1: Get Your Qdrant Cloud Credentials

1. Go to https://cloud.qdrant.io and create a free account
2. Create a new cluster (free tier available)
3. Once created, go to **Settings** or **Cluster Details**
4. Copy:
   - **Cluster URL** (looks like `https://xxxxx-qdrant.a.run.app:6333`)
   - **API Key** (long alphanumeric string)

## Step 2: Get Your Gemini API Key

1. Go to https://ai.google.dev
2. Click **Get API key** in the top-right
3. Create a new API key (free tier available)
4. Copy the key

## Step 3: Set Up Python Virtual Environment

From the repo root:

```bash
# Create virtual environment
python -m venv .venv

# Activate it
# On Windows (cmd):
.venv\Scripts\activate
# On Windows (PowerShell):
.venv\Scripts\Activate.ps1
# On macOS/Linux:
source .venv/bin/activate
```

## Step 4: Install Dependencies

```bash
pip install -r backend/requirements.txt
```

## Step 5: Create `.env` File

Create a `.env` file in the repo root with your credentials:

```
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_MODEL=gemini-pro
QDRANT_URL=https://your-cluster-url.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

**Example (with real values):**
```
GEMINI_API_KEY=AIzaSyD_Example_Key_123456789
GEMINI_MODEL=gemini-pro
QDRANT_URL=https://my-cluster-abc123.qdrant.io:6333
QDRANT_API_KEY=ey4321_example_qdrant_key_9876
```

## Step 6: Populate Qdrant with Book Content

Make sure your virtual environment is still active, then:

```bash
python backend/scripts/populate_qdrant.py
```

This will:
- Load all markdown files from `frontend/docs/book/`
- Compute embeddings using `sentence-transformers`
- Upload vectors to your Qdrant Cloud cluster
- Print progress and status

**Expected output:**
```
Loading markdown documents from frontend/docs/book...
Found XX documents. Generating embeddings (this may take time)...
Preparing metadata and upserting into Qdrant...
Upserted XX documents to collection 'textbook_chapters'.
Upsert complete. You can now query the backend chat endpoint.
```

## Step 7: Run the Backend

```bash
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Started server process [XXXXX]
INFO:     Application startup complete
INFO:     Uvicorn running on http://0.0.0.0:8000
```

## Step 8: Test the Backend

### Health Check
```bash
curl http://localhost:8000/health
# Expected: {"status":"ok"}
```

### Chat Endpoint
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "user_level": "Beginner"}'
```

### Expected Response
```json
{
  "response": "Physical AI refers to...",
  "source_documents": [
    {
      "id": "doc_0",
      "title": "Introduction to Physical AI",
      "score": 0.87
    }
  ]
}
```

## Troubleshooting

### "Connection refused" or "Cannot connect to Qdrant"
- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Verify your Qdrant Cloud cluster is running
- Test URL manually: `curl https://your-cluster-url.qdrant.io/health`

### "google.generativeai not found"
- Ensure `google-generativeai` is installed: `pip list | grep google`
- Reinstall: `pip install google-generativeai`

### "GEMINI_API_KEY not configured" (gets dummy LLM response)
- Check `.env` file exists in repo root
- Verify `GEMINI_API_KEY` is set correctly
- Make sure the backend was restarted after `.env` was created

### Embedding generation is slow
- This is normal the first time (downloads `sentence-transformers` model)
- Subsequent runs reuse cached embeddings
- On a typical machine, expect 1-2 minutes for 70+ documents

### "Collection not found" error
- Run `python backend/scripts/populate_qdrant.py` first
- Check Qdrant Cloud dashboard to verify collection was created

## File Structure

```
backend/
  src/
    main.py                          # FastAPI app (loads .env)
    services/
      vector_store.py                # Qdrant client (supports cloud + localhost)
      rag_service.py                 # RAG pipeline (uses Gemini LLM)
      content_loader.py              # Loads markdown from frontend/docs/book
      embedding_generator.py         # sentence-transformers embeddings
    api/
      chat.py                        # /chat endpoint
      translate.py                   # /translate endpoint
  scripts/
    populate_qdrant.py               # Populate Qdrant from markdown
  requirements.txt                   # Python dependencies (includes google-generativeai)

.env                                 # YOUR CREDENTIALS (create this)
.env.example                         # Template (reference)
```

## Next Steps

- Frontend chatbot at `frontend/src/components/Chatbot/index.tsx` calls `http://localhost:8000/chat`
- Translation button at `frontend/src/components/TranslationButton/index.tsx` calls `http://localhost:8000/translate`
- To test frontend, run `npm run start` from `frontend/` directory (requires separate terminal)

## Quick Copy-Paste Setup (if you have credentials ready)

```bash
# 1. Create venv and activate
python -m venv .venv
.venv\Scripts\activate  # Windows cmd
# Or: .venv\Scripts\Activate.ps1  # Windows PowerShell

# 2. Install deps
pip install -r backend/requirements.txt

# 3. Create .env (copy and edit with YOUR credentials)
copy .env.example .env
# Edit .env in your editor and add real values

# 4. Populate Qdrant
python backend/scripts/populate_qdrant.py

# 5. Run backend
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000
```

That's it! Your chatbot is ready.
