import requests
import time

time.sleep(5)  # Wait for server to start

print("Testing health...")
r = requests.get('http://localhost:8000/health')
print('Health:', r.json())

print("Testing chat...")
r = requests.post('http://localhost:8000/chat', json={'query': 'What is a robot?', 'user_level': 'Beginner'})
print('Response:', r.json())