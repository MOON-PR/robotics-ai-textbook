@echo off
REM Quick-start script for backend setup (Windows)
REM Usage: Run from repo root
REM   setup_backend.bat

setlocal enabledelayedexpansion

echo.
echo ===== Robotics Course Book - Backend Setup (No Docker) =====
echo.

REM Check Python
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found. Install Python 3.9+ and add to PATH.
    pause
    exit /b 1
)
echo ✓ Python found
echo.

REM Create venv if it doesn't exist
if not exist ".venv" (
    echo Creating Python virtual environment...
    python -m venv .venv
    if errorlevel 1 (
        echo ERROR: Failed to create virtual environment.
        pause
        exit /b 1
    )
    echo ✓ Virtual environment created
) else (
    echo ✓ Virtual environment already exists
)
echo.

REM Activate venv
echo Activating virtual environment...
call .venv\Scripts\activate.bat
if errorlevel 1 (
    echo ERROR: Failed to activate virtual environment.
    pause
    exit /b 1
)
echo ✓ Virtual environment activated
echo.

REM Install dependencies
echo Installing dependencies...
pip install -q -r backend/requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies.
    pause
    exit /b 1
)
echo ✓ Dependencies installed
echo.

REM Check .env file
if not exist ".env" (
    echo Creating .env file from template...
    copy .env.example .env >nul
    echo.
    echo ⚠️  .env file created. EDIT IT NOW with your credentials:
    echo.
    echo   GEMINI_API_KEY=your_key_here
    echo   GEMINI_MODEL=gemini-pro
    echo   QDRANT_URL=https://your-cluster.qdrant.io:6333
    echo   QDRANT_API_KEY=your_api_key_here
    echo.
    echo Open .env in your editor and fill in the values, then run this script again.
    pause
    exit /b 0
) else (
    echo ✓ .env file exists
)
echo.

REM Validate env
echo Validating configuration...
python backend/scripts/validate_env.py
if errorlevel 1 (
    pause
    exit /b 1
)
echo.

REM Ask to populate
set /p populate="Ready to populate Qdrant? (y/n) "
if /i "!populate!"=="y" (
    echo.
    echo Populating Qdrant (this may take 2-5 minutes)...
    python backend/scripts/populate_qdrant.py
    if errorlevel 1 (
        echo ERROR: Failed to populate Qdrant.
        pause
        exit /b 1
    )
    echo ✓ Qdrant populated
    echo.
)

REM Start backend
echo.
echo Starting FastAPI backend on http://localhost:8000
echo Press Ctrl+C to stop.
echo.
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000

pause
