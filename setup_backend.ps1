#!/usr/bin/env pwsh
# Quick-start script for backend setup (Windows PowerShell)
# Usage: Run from repo root
#   .\setup_backend.ps1

$ErrorActionPreference = "Stop"

function Write-Success { Write-Host "✓ $args" -ForegroundColor Green }
function Write-Warning { Write-Host "⚠️  $args" -ForegroundColor Yellow }
function Write-Error { Write-Host "❌ $args" -ForegroundColor Red }

Write-Host ""
Write-Host "===== Robotics Course Book - Backend Setup (No Docker) =====" -ForegroundColor Cyan
Write-Host ""

# Check Python
try {
    $pythonVersion = python --version 2>&1
    Write-Success "Python found: $pythonVersion"
} catch {
    Write-Error "Python not found. Install Python 3.9+ and add to PATH."
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Host ""

# Create venv if it doesn't exist
if (Test-Path ".venv") {
    Write-Success "Virtual environment already exists"
} else {
    Write-Host "Creating Python virtual environment..."
    python -m venv .venv
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Failed to create virtual environment."
        Read-Host "Press Enter to exit"
        exit 1
    }
    Write-Success "Virtual environment created"
}
Write-Host ""

# Activate venv
Write-Host "Activating virtual environment..."
& ".\.venv\Scripts\Activate.ps1"
Write-Success "Virtual environment activated"
Write-Host ""

# Install dependencies
Write-Host "Installing dependencies..."
pip install -q -r backend/requirements.txt
if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to install dependencies."
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Success "Dependencies installed"
Write-Host ""

# Check .env file
if (Test-Path ".env") {
    Write-Success ".env file exists"
} else {
    Write-Host "Creating .env file from template..."
    Copy-Item ".env.example" ".env"
    Write-Host ""
    Write-Warning ".env file created. EDIT IT NOW with your credentials:"
    Write-Host ""
    Write-Host "  GEMINI_API_KEY=your_key_here"
    Write-Host "  GEMINI_MODEL=gemini-pro"
    Write-Host "  QDRANT_URL=https://your-cluster.qdrant.io:6333"
    Write-Host "  QDRANT_API_KEY=your_api_key_here"
    Write-Host ""
    Write-Host "Open .env in your editor and fill in the values, then run this script again."
    Read-Host "Press Enter to exit"
    exit 0
}
Write-Host ""

# Validate env
Write-Host "Validating configuration..."
python backend/scripts/validate_env.py
if ($LASTEXITCODE -ne 0) {
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Host ""

# Ask to populate
$populate = Read-Host "Ready to populate Qdrant? (y/n)"
if ($populate -eq "y" -or $populate -eq "Y") {
    Write-Host ""
    Write-Host "Populating Qdrant (this may take 2-5 minutes)..."
    python backend/scripts/populate_qdrant.py
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Failed to populate Qdrant."
        Read-Host "Press Enter to exit"
        exit 1
    }
    Write-Success "Qdrant populated"
    Write-Host ""
}

# Start backend
Write-Host ""
Write-Host "Starting FastAPI backend on http://localhost:8000" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop."
Write-Host ""
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000

Read-Host "Press Enter to exit"
