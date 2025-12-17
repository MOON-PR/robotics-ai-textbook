# Backend Chatbot - Complete Setup Guide Index

## 📖 Read These Files (in order)

1. **`BACKEND_COMPLETE.txt`** (Start here!)
   - Summary of what's done
   - Quick 5-minute setup
   - Troubleshooting table

2. **`BACKEND_GEMINI_QDRANT.md`** (Detailed guide)
   - Architecture overview
   - Step-by-step setup
   - Testing instructions
   - FAQ

3. **`SETUP_BACKEND.md`** (Very detailed)
   - Prerequisites
   - How to get API keys
   - Full setup walkthrough
   - Troubleshooting with solutions

## 🚀 Quick Start (Pick One)

### Option A: Windows Batch Script (Easiest)
```cmd
setup_backend.bat
```
Handles venv creation, dependency install, and starts backend.

### Option B: Windows PowerShell Script
```powershell
.\setup_backend.ps1
```
Same as batch but with prettier output.

### Option C: Manual (Linux/macOS/Custom)
See `BACKEND_GEMINI_QDRANT.md` → "Quick Copy-Paste Setup"

## 🔑 Get These API Keys (Both Free)

### Gemini API Key
1. Go to https://ai.google.dev
2. Click "Get API key" 
3. Copy the key
4. Add to `.env` as `GEMINI_API_KEY`

### Qdrant Cloud
1. Go to https://cloud.qdrant.io
2. Create account + cluster
3. Get URL and API key from Settings
4. Add to `.env` as `QDRANT_URL` and `QDRANT_API_KEY`

## 📝 Create `.env` File

Copy `.env.example` to `.env` and fill in your keys:
```ini
GEMINI_API_KEY=your_gemini_key_here
GEMINI_MODEL=gemini-pro
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_key_here
```

## ✅ What Gets Set Up

- **Backend API** on `http://localhost:8000`
- **Chat endpoint** (`/chat`) - RAG-based Q&A using Gemini
- **Translate endpoint** (`/translate`) - Translate to Urdu using Gemini
- **Vector DB** - Qdrant Cloud with your book content
- **Embeddings** - Sentence-transformers (all-MiniLM-L6-v2)

## 📂 Files Created/Modified

### New Files
- `.env.example` - Template for environment variables
- `.env` - Your actual credentials (you create this)
- `BACKEND_COMPLETE.txt` - This summary
- `BACKEND_GEMINI_QDRANT.md` - Full guide
- `SETUP_BACKEND.md` - Detailed walkthrough
- `setup_backend.bat` - Automated setup (Windows batch)
- `setup_backend.ps1` - Automated setup (Windows PowerShell)
- `backend/scripts/validate_env.py` - Config validation script

### Modified Files
- `backend/src/services/vector_store.py` - Qdrant Cloud support
- `backend/src/main.py` - Load `.env` file
- `backend/requirements.txt` - Added google-generativeai, python-dotenv

### Already Working (No changes needed)
- `backend/src/services/rag_service.py` - RAG pipeline
- `backend/src/api/chat.py` - Chat endpoint
- `backend/src/api/translate.py` - Translation endpoint
- `backend/scripts/populate_qdrant.py` - Load book data
- `backend/src/services/content_loader.py` - Reads markdown
- `backend/src/services/embedding_generator.py` - Creates vectors

## 🧪 Test Commands

Once backend is running:

```bash
# Health check
curl http://localhost:8000/health

# Ask question
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "user_level": "Beginner"}'

# Translate
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"text": "Hello world", "target_language": "Urdu"}'
```

## 🎯 Next Steps

1. **Get API keys** → Gemini (https://ai.google.dev) + Qdrant (https://cloud.qdrant.io)
2. **Run setup** → `setup_backend.bat` or manual steps from BACKEND_GEMINI_QDRANT.md
3. **Test backend** → Use curl commands above
4. **Run frontend** → `cd frontend && npm run start`
5. **Use chatbot** → Open http://localhost:3000 in browser

## ❓ FAQ

**Q: Do I need Docker?**  
A: No! Qdrant Cloud handles the vector database.

**Q: Is this free?**  
A: Yes! Both Gemini API and Qdrant Cloud have free tiers.

**Q: Can I use local Qdrant?**  
A: Yes, set `QDRANT_HOST=localhost` and `QDRANT_PORT=6333` in `.env` instead of cloud URL/key.

**Q: How do I know if it worked?**  
A: Run the test commands above. You should get JSON responses from the backend.

**Q: Can I add Ollama for local LLM?**  
A: Yes! LMK and I can add Ollama support as a fallback.

## 📞 Troubleshooting

**Backend won't start?**
- Run: `python backend/scripts/validate_env.py`
- Check: Is `.env` file present and filled out?

**"Connection refused" error?**
- Check: Is Qdrant Cloud cluster URL correct?
- Test: Visit `https://your-cluster-url.qdrant.io/health` in browser

**"LLM not configured" response?**
- Check: Is `GEMINI_API_KEY` in `.env`?
- Restart: Kill and restart the backend after editing `.env`

**Qdrant population stuck?**
- This is normal for first run (2-5 minutes)
- Subsequent runs skip population step

---

## 🎉 You're All Set!

Follow one of the Quick Start options above and your chatbot will be running in < 10 minutes.

Questions? Check the detailed guides above or review the code comments.

Good luck! 🚀
