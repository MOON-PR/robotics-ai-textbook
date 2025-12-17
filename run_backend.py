import os
os.environ['PYTHONPATH'] = r'C:\Users\Rohaan Computers\Desktop\Hackathon Q4\Robotics-Course-Book - Copy'
import uvicorn
uvicorn.run('backend.src.main:app', host='0.0.0.0', port=8000)