import requests
import os
from dotenv import load_dotenv

load_dotenv()

# Set up OpenRouter
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
headers = {
    "Authorization": f"Bearer {openrouter_api_key}",
    "Content-Type": "application/json"
}

# Test the API with a simple request
data = {
    "model": "openai/gpt-3.5-turbo",
    "messages": [{"role": "user", "content": "Hello"}],
    "max_tokens": 10
}

try:
    response = requests.post("https://openrouter.ai/api/v1/chat/completions", headers=headers, json=data)
    print("Status Code:", response.status_code)
    print("Response:", response.text[:500])  # First 500 chars
except Exception as e:
    print("Error:", str(e))