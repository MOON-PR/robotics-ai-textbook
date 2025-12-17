from fastapi import APIRouter
from pydantic import BaseModel
from typing import Dict, Optional

router = APIRouter()

# In-memory dummy store for user profiles
# In a real application, this would be a database
_user_profiles: Dict[str, str] = {} # username: level

class ProfileRequest(BaseModel):
    username: str
    level: str

class ProfileResponse(BaseModel):
    username: str
    level: str

@router.get("/profile/{username}", response_model=ProfileResponse)
async def get_user_profile(username: str):
    """
    Retrieve user profile (learning level).
    """
    level = _user_profiles.get(username, "Beginner") # Default to Beginner if not found
    return ProfileResponse(username=username, level=level)

@router.post("/profile", response_model=ProfileResponse)
async def update_user_profile(request: ProfileRequest):
    """
    Update user profile (learning level).
    """
    _user_profiles[request.username] = request.level
    return ProfileResponse(username=request.username, level=request.level)
