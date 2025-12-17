# Backend Setup Checklist

Complete these steps in order to get your chatbot running with Gemini + Qdrant Cloud.

## 🔑 Step 1: Get API Keys

- [ ] **Gemini API Key**
  - [ ] Go to https://ai.google.dev
  - [ ] Click "Get API key"
  - [ ] Copy the key
  - [ ] Note: `GEMINI_API_KEY=...`

- [ ] **Qdrant Cloud Cluster**
  - [ ] Go to https://cloud.qdrant.io
  - [ ] Create account (or sign in)
  - [ ] Create a new cluster (free tier)
  - [ ] Go to Settings/Cluster Details
  - [ ] Copy Cluster URL → Note: `QDRANT_URL=https://...`
  - [ ] Copy API Key → Note: `QDRANT_API_KEY=...`

## 📝 Step 2: Configure Environment

- [ ] Copy `.env.example` to `.env`
  ```cmd
  copy .env.example .env
  ```

- [ ] Edit `.env` in your text editor with your credentials:
  ```ini
  GEMINI_API_KEY=your_actual_key_here
  GEMINI_MODEL=gemini-pro
  QDRANT_URL=https://your-cluster.qdrant.io:6333
  QDRANT_API_KEY=your_actual_key_here
  ```

- [ ] Save `.env` file

## 🚀 Step 3: Run Setup Script

**Choose ONE:**

### Option A: Windows Batch (Easiest)
```cmd
setup_backend.bat
```
- [ ] Double-click `setup_backend.bat` from Windows Explorer, OR
- [ ] Run from Command Prompt: `setup_backend.bat`
- [ ] Script will: create venv, install deps, validate config, populate Qdrant, start backend
- [ ] Watch for the message: "✓ Qdrant populated"
- [ ] Watch for the message: "Uvicorn running on http://0.0.0.0:8000"

### Option B: Windows PowerShell
```powershell
.\setup_backend.ps1
```
- [ ] Right-click `setup_backend.ps1` → "Run with PowerShell", OR
- [ ] Open PowerShell and run: `.\setup_backend.ps1`
- [ ] Same as batch script above

### Option C: Manual Setup (if scripts don't work)
```cmd
REM Create venv
python -m venv .venv

REM Activate venv
.venv\Scripts\activate

REM Install dependencies
pip install -r backend/requirements.txt

REM Validate config
python backend/scripts/validate_env.py

REM Populate Qdrant (takes 2-5 minutes first time)
python backend/scripts/populate_qdrant.py

REM Start backend
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000
```

- [ ] Python venv created
- [ ] Python venv activated
- [ ] Dependencies installed (pip install succeeded)
- [ ] Config validated (validate_env.py ran without errors)
- [ ] Qdrant populated (saw "Upserted XX documents" message)
- [ ] Backend started (saw "Uvicorn running on http://0.0.0.0:8000")

## ✅ Step 4: Verify Backend Works

While backend is running, open a NEW terminal/command prompt and test:

```bash
# Test 1: Health check
curl http://localhost:8000/health
# Expected: {"status":"ok"}

# Test 2: Ask a question
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d "{\"query\": \"What is Physical AI?\", \"user_level\": \"Beginner\"}"
# Expected: JSON response with "response" and "source_documents"

# Test 3: Translate
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d "{\"text\": \"Hello world\", \"target_language\": \"Urdu\"}"
# Expected: JSON response with translated text
```

- [ ] Health check returns `{"status":"ok"}`
- [ ] Chat endpoint returns a response (not placeholder)
- [ ] Translate endpoint returns translated text
- [ ] All tests passed!

## 🎨 Step 5: Test Frontend (Optional)

Open another terminal and run:

```bash
cd frontend
npm run start
```

- [ ] Frontend starts on http://localhost:3000
- [ ] Homepage loads (no 404)
- [ ] Chatbot sidebar is visible
- [ ] Can type in chatbot (sends to backend `/chat`)
- [ ] Translation button works (sends to backend `/translate`)

## 🎉 Done!

You now have a fully functional chatbot with:
- ✅ Gemini LLM for answers
- ✅ Qdrant Cloud vector DB
- ✅ RAG pipeline from your book content
- ✅ Frontend integration

## 🔧 Troubleshooting

If something goes wrong, check:

1. **"Connection refused"**
   - Is Qdrant Cloud running? Check dashboard at https://cloud.qdrant.io
   - Is the URL correct in `.env`?

2. **"google.generativeai not found"**
   - Run: `pip install google-generativeai`

3. **"GEMINI_API_KEY not configured" (dummy response)**
   - Is `.env` present in repo root?
   - Does it have `GEMINI_API_KEY=...`?
   - Restart backend after editing `.env`

4. **"Collection not found" error**
   - Run: `python backend/scripts/populate_qdrant.py`
   - Wait for completion

5. **Script won't run (permission denied)**
   - Windows batch: Try running from Command Prompt
   - Windows PowerShell: Try `powershell -ExecutionPolicy Bypass -File setup_backend.ps1`

## 📖 More Help

- **Questions?** → Read `README_BACKEND.md`
- **Detailed steps?** → Read `SETUP_BACKEND.md`
- **Architecture?** → Read `BACKEND_GEMINI_QDRANT.md`
- **Code?** → Check `backend/src/services/rag_service.py` and `backend/src/main.py`

---

Good luck! 🚀

Check off boxes as you complete each step. Feel free to skip "Optional" sections.
