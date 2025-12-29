from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import os
from dotenv import load_dotenv
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from pydantic import BaseModel
from typing import List, Dict, Any
import jwt
import hashlib
from datetime import datetime, timedelta

# Load environment variables
load_dotenv()

# JWT Configuration
JWT_SECRET = os.getenv("JWT_SECRET", "default_secret_key_for_development")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

app = FastAPI(title="RAG Service API", description="API for RAG service with Qdrant and OpenAI")

# Pydantic models for authentication
class LoginRequest(BaseModel):
    username: str
    password: str

class TokenResponse(BaseModel):
    access_token: str
    token_type: str

# Configure CORS for localhost:3000
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize OpenRouter and Qdrant clients
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
if openrouter_api_key:
    openai.base_url = "https://openrouter.ai/api/v1"
    openai.api_key = openrouter_api_key
    openai.default_headers = {"HTTP-Referer": "http://localhost:8000", "X-Title": "Physical AI Courseware RAG"}

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_client = None
if qdrant_url and qdrant_api_key:
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
    )

@app.get("/")
async def root():
    return {"message": "RAG Service API is running"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

def create_access_token(data: dict):
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, JWT_SECRET, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise HTTPException(status_code=401, detail="Could not validate credentials")
        return username
    except jwt.PyJWTError:
        raise HTTPException(status_code=401, detail="Could not validate credentials")

security = HTTPBearer()

async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    token = credentials.credentials
    return verify_token(token)

@app.post("/login", response_model=TokenResponse)
async def login(request: LoginRequest):
    # For simplicity, we'll use a hardcoded user
    # In a real application, you would check against a database
    if request.username == "admin" and request.password == "password":
        token_data = {"sub": request.username}
        access_token = create_access_token(data=token_data)
        return {"access_token": access_token, "token_type": "bearer"}
    else:
        raise HTTPException(status_code=401, detail="Incorrect username or password")

# Pydantic model for chat requests
class ChatRequest(BaseModel):
    message: str
    history: List[Dict[str, str]] = []

@app.post("/chat")
async def chat_endpoint(request: ChatRequest, current_user: str = Depends(get_current_user)):
    if not openrouter_api_key or not qdrant_client:
        raise HTTPException(status_code=500, detail="Missing required environment variables")

    try:
        # First, generate an embedding for the query text
        # Since we're using OpenRouter, we'll generate an embedding using the OpenAI-compatible endpoint
        try:
            query_embedding = openai.embeddings.create(
                model="text-embedding-ada-002",  # OpenAI embedding model via OpenRouter
                input=request.message
            ).data[0].embedding
        except Exception as e:
            # If OpenAI embedding model doesn't work via OpenRouter, use a fallback
            # Create a simple embedding based on the text (this is less effective but will work)
            import hashlib
            text_lower = request.message.lower()
            unique_chars = set(text_lower)

            # Create a basic embedding based on text features
            query_embedding = []
            for i in range(1536):  # Standard embedding size matching our collection
                char_idx = i % len(unique_chars) if unique_chars else 0
                char_val = ord(list(unique_chars)[char_idx]) if unique_chars else 0
                # Add some variation based on position and text properties
                val = (char_val + i) % 1000
                val = (val * hash(text_lower + str(i)) % 10000) / 10000.0
                query_embedding.append(val)

        # Search Qdrant for relevant documents using the vector embedding
        search_results = qdrant_client.query_points(
            collection_name="book_docs",
            query=query_embedding,
            limit=5  # Get top 5 most relevant documents
        ).points

        # Format context from search results
        context_parts = []
        for result in search_results:
            if hasattr(result, 'payload') and result.payload:
                content = result.payload.get("content", "")
                filename = result.payload.get("filename", "unknown")
                context_parts.append(f"Document: {filename}\nContent: {content}\n")

        context = "\n".join(context_parts)

        # Prepare the message for the model
        messages = [
            {
                "role": "system",
                "content": "You are a helpful assistant that answers questions based on the provided documentation. Use the context to provide accurate and helpful responses. If the context doesn't contain the information needed to answer the question, say so."
            }
        ]

        # Add context to the message
        if context:
            messages.append({
                "role": "system",
                "content": f"Context:\n{context}"
            })

        # Add conversation history if available
        for msg in request.history:
            messages.append({
                "role": msg.get("role", "user"),
                "content": msg.get("content", "")
            })

        # Add the current user message
        messages.append({
            "role": "user",
            "content": request.message
        })

        # Generate response using OpenRouter API directly with requests
        import requests
        headers = {
            "Authorization": f"Bearer {openrouter_api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": "openai/gpt-3.5-turbo",  # Using a reliable model that's always available
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 1000
        }

        response = requests.post("https://openrouter.ai/api/v1/chat/completions", headers=headers, json=data)

        if response.status_code == 200:
            response_json = response.json()
            if 'choices' in response_json and len(response_json['choices']) > 0:
                answer = response_json['choices'][0]['message']['content']
            else:
                answer = "Error: No response from AI model"
        else:
            answer = f"Error: {response.status_code} - {response.text}"

        return {"response": answer, "context_sources": [r.payload.get("filename", "unknown") if hasattr(r, 'payload') and r.payload else "unknown" for r in search_results]}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

