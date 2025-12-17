# Backend Chatbot Setup - Gemini + Qdrant Cloud (No Docker)

This guide gets your backend chatbot running with **Gemini API** and **Qdrant Cloud** (no Docker required).

## What You'll Get

- ✅ FastAPI backend running locally
- ✅ Gemini LLM for intelligent answers
- ✅ Qdrant Cloud vector database (free tier)
- ✅ RAG pipeline (Retrieval Augmented Generation) from your book
- ✅ Chat and translate endpoints for the frontend

## Quick Start (5 minutes if you have API keys)

### 1️⃣ Get API Keys (free)

**Gemini API Key** (free tier):
- Go to https://ai.google.dev
- Click "Get API key"
- Copy the key

**Qdrant Cloud** (free tier):
- Go to https://cloud.qdrant.io
- Sign up and create a cluster
- Copy the **Cluster URL** and **API Key** from Settings

### 2️⃣ Setup Backend (Windows)

From the repo root, double-click:
```
setup_backend.bat
```

This script will:
1. Create a Python virtual environment
2. Install dependencies
3. Create `.env` file
4. Validate your configuration
5. Populate Qdrant with book content
6. Start the backend

**If running manually:**

```cmd
REM Create and activate venv
python -m venv .venv
.venv\Scripts\activate

REM Install dependencies
pip install -r backend/requirements.txt

REM Create .env with your credentials
copy .env.example .env
REM Edit .env in your editor!

REM Validate config
python backend/scripts/validate_env.py

REM Populate Qdrant (one-time, takes 2-5 min)
python backend/scripts/populate_qdrant.py

REM Start backend
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000
```

### 3️⃣ Test It

Once the backend is running, test it:

```bash
# Health check
curl http://localhost:8000/health

# Ask a question
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "user_level": "Beginner"}'
```

Expected response:
```json
{
  "response": "Physical AI refers to...",
  "source_documents": [{"id": "doc_0", "title": "...", "score": 0.85}]
}
```

### 4️⃣ Connect Frontend

The frontend chatbot is already configured to call `http://localhost:8000/chat`.

To test the full UI, open another terminal and run:

```bash
cd frontend
npm run start
```

Then open http://localhost:3000 and try the chatbot in the sidebar!

---

## File Reference

**Changed Files:**
- `backend/src/services/vector_store.py` - Now supports Qdrant Cloud (URL + API key)
- `backend/src/main.py` - Loads `.env` using `python-dotenv`
- `backend/requirements.txt` - Added `google-generativeai` and `python-dotenv`

**New Files:**
- `.env.example` - Template with all required env vars
- `.env` - **YOU CREATE THIS** with your actual credentials
- `SETUP_BACKEND.md` - Detailed setup guide
- `setup_backend.bat` - Automated setup script for Windows
- `backend/scripts/validate_env.py` - Check config before startup
- `backend/scripts/populate_qdrant.py` - Load book into Qdrant (uses existing code)

---

## Troubleshooting

### "Connection refused" → Can't connect to Qdrant
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Test URL in browser: `https://your-cluster-url.qdrant.io/health`
- Check Qdrant Cloud dashboard to ensure cluster is running

### "google.generativeai not found"
- Run: `pip install google-generativeai`

### "LLM not configured" (dummy response)
- Check `.env` exists in repo root
- Verify `GEMINI_API_KEY` is not empty
- Restart the backend after editing `.env`

### Qdrant population is slow
- This is normal (downloads embeddings model + computes vectors)
- Expect 2-5 minutes for 70+ documents on typical CPU
- Only runs once; subsequent starts are instant

### "Collection not found" error
- Run `python backend/scripts/populate_qdrant.py` first
- Check Qdrant Cloud dashboard for the `textbook_chapters` collection

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Frontend (React/Docusaurus)                    │
│  - Chatbot UI → POST /chat                      │
│  - Translation UI → POST /translate             │
└────────────────┬────────────────────────────────┘
                 │ http://localhost:8000
┌────────────────▼────────────────────────────────┐
│  Backend (FastAPI)                              │
│  - /chat → RAGService.ask_question()            │
│  - /translate → Gemini translation              │
│  - Startup: Initialize RAG pipeline             │
└────────────────┬────────────────────────────────┘
                 │
    ┌────────────┴────────────┐
    │                         │
┌───▼─────────────┐  ┌────────▼──────────┐
│ Gemini API      │  │ Qdrant Cloud      │
│ (LLM)           │  │ (Vector DB)       │
│ Responses       │  │ Book embeddings   │
└─────────────────┘  └───────────────────┘
```

---

## Next Steps

1. ✅ **Backend running?** → Test `/chat` endpoint
2. ✅ **Frontend running?** → Try chatbot in sidebar
3. 🚀 **Want Ollama too?** → We can add local LLM fallback (optional)
4. 📦 **Deploy?** → Deploy backend to cloud (Vercel, Railway, Render, etc.)

---

## FAQ

**Q: Do I need Docker?**  
A: No! This setup uses Qdrant Cloud (no server to manage) and runs the backend directly on your machine.

**Q: Is Qdrant Cloud free?**  
A: Yes, free tier allows up to 1GB vectors + 100 requests/month. Perfect for testing.

**Q: Can I use local Qdrant instead of Cloud?**  
A: Yes! Just set `QDRANT_HOST=localhost` and `QDRANT_PORT=6333` in `.env` and run local Qdrant (Docker or standalone).

**Q: Will this work offline?**  
A: No, you need internet for:
- Gemini API calls (LLM)
- Qdrant Cloud connections (vector DB)
- Downloading models on first run

**Q: Can I add Ollama for local LLM?**  
A: Yes! We can add Ollama support as fallback. Ask and I'll add it.

---

## Support

If you run into issues:
1. Run `python backend/scripts/validate_env.py` to check config
2. Check `SETUP_BACKEND.md` for detailed troubleshooting
3. Review the logs from uvicorn for specific errors

Good luck! 🚀
