#!/usr/bin/env python
"""
Validate that all required environment variables are set for backend startup.
Run this before starting uvicorn to catch configuration issues early.

Usage:
    python backend/scripts/validate_env.py
"""
import os
import sys
from pathlib import Path

def validate_env():
    """Check all required env vars and warn about missing ones."""
    errors = []
    warnings = []
    
    # Check Gemini
    gemini_key = os.environ.get("GEMINI_API_KEY")
    gemini_model = os.environ.get("GEMINI_MODEL", "gemini-pro")
    
    if not gemini_key:
        warnings.append("⚠️  GEMINI_API_KEY not set. LLM will fall back to dummy (placeholder responses).")
    else:
        print(f"✓ GEMINI_API_KEY is set (model: {gemini_model})")
    
    # Check Qdrant (either cloud or localhost)
    qdrant_url = os.environ.get("QDRANT_URL")
    qdrant_api_key = os.environ.get("QDRANT_API_KEY")
    qdrant_host = os.environ.get("QDRANT_HOST", "localhost")
    qdrant_port = os.environ.get("QDRANT_PORT", "6333")
    
    if qdrant_url and qdrant_api_key:
        print(f"✓ Qdrant Cloud configured: {qdrant_url}")
    else:
        print(f"✓ Qdrant Localhost configured: {qdrant_host}:{qdrant_port}")
    
    # Check frontend/docs/book exists
    docs_path = Path(__file__).resolve().parents[2] / "frontend" / "docs" / "book"
    if docs_path.exists():
        md_files = list(docs_path.glob("**/*.md"))
        print(f"✓ Found {len(md_files)} markdown files in {docs_path}")
    else:
        warnings.append(f"⚠️  frontend/docs/book not found at {docs_path}. Populate script may fail.")
    
    print()
    if warnings:
        for w in warnings:
            print(w)
        print()
    
    if errors:
        print("❌ Configuration errors:")
        for e in errors:
            print(f"  {e}")
        sys.exit(1)
    else:
        print("✅ Environment configuration looks good!")
        print()
        print("Next steps:")
        print("  1. Run: python backend/scripts/populate_qdrant.py")
        print("  2. Then: uvicorn backend.src.main:app --reload --port 8000")
        print()

if __name__ == "__main__":
    validate_env()
