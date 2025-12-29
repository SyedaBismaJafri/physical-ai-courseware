import openai
import os
from dotenv import load_dotenv

load_dotenv()

# Set up OpenRouter
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
openai.base_url = "https://openrouter.ai/api/v1"
openai.api_key = openrouter_api_key

# Test the API with a simple request
try:
    response = openai.chat.completions.create(
        model="openai/gpt-3.5-turbo",
        messages=[{"role": "user", "content": "Hello"}],
        max_tokens=10
    )
    print("Success:", response.choices[0].message.content)
except Exception as e:
    print("Error:", str(e))
    print("Error type:", type(e))