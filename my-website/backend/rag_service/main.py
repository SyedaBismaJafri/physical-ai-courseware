from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import os
from dotenv import load_dotenv
from pydantic import BaseModel
from typing import List, Dict, Any
import jwt
from datetime import datetime, timedelta
import asyncio
from pydantic_ai import Agent, RunContext
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
import requests


# Load environment variables
load_dotenv()

# JWT Configuration
JWT_SECRET = os.getenv("JWT_SECRET", "default_secret_key_for_development")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

app = FastAPI(title="Multi-Agent RAG Service API", description="API for Multi-Agent RAG service with Qdrant and OpenAI")

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
    # Also set the OpenAI API key environment variable to use OpenRouter
    import os
    os.environ["OPENAI_API_KEY"] = openrouter_api_key

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
    return {"message": "Multi-Agent RAG Service API is running"}

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


# Define the tools using pydantic-ai
def fetch_qdrant_context(query: str) -> str:
    """Search the knowledge base using Qdrant for relevant information."""
    if not qdrant_client:
        return "Error: Qdrant client not initialized"

    try:
        # Generate an embedding for the query text
        query_embedding = openai.embeddings.create(
            model="text-embedding-ada-002",  # OpenAI embedding model via OpenRouter
            input=query
        ).data[0].embedding
    except Exception as e:
        # Fallback to a simple embedding if OpenAI fails
        import hashlib
        text_lower = query.lower()
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

    try:
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
        return context if context else "No relevant information found in the knowledge base."
    except Exception as e:
        return f"I couldn't access the knowledge base right now: {str(e)}"


def format_code_block(code: str) -> str:
    """Format raw code into proper Markdown with language detection."""
    # Simple language detection based on common patterns
    code_lower = code.lower()

    # Detect language based on common keywords
    if any(lang in code_lower for lang in ['def ', 'import ', 'class ', 'print(', 'lambda']):
        language = 'python'
    elif any(lang in code_lower for lang in ['function ', 'var ', 'let ', 'const ', '{', '}']):
        language = 'javascript'
    elif any(lang in code_lower for lang in ['public class', 'private', 'static', 'void main']):
        language = 'java'
    elif any(lang in code_lower for lang in ['#include', 'int main', 'printf']):
        language = 'c'
    elif any(lang in code_lower for lang in ['<html', '<body', '<div', '<head']):
        language = 'html'
    elif any(lang in code_lower for lang in ['<', 'class=', 'id=', 'style=']):
        language = 'xml'
    elif any(lang in code_lower for lang in ['SELECT ', 'FROM ', 'WHERE ', 'INSERT INTO']):
        language = 'sql'
    else:
        language = 'python'  # Default to Python

    # Format the code block
    formatted_code = f"```{language}\n{code}\n```"
    return formatted_code


def get_current_time() -> str:
    """Get the current date and time."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# Create the Research Agent
research_agent = Agent(
    model="openai:gpt-3.5-turbo",  # Using OpenRouter model via environment config
    system_prompt="You are a research expert. Use the fetch_qdrant_context tool to find relevant information for the user's query. Provide detailed, accurate information based on the retrieved context.",
    tools=[fetch_qdrant_context, get_current_time]
)


# Create the Coding Agent
coding_agent = Agent(
    model="openai:gpt-3.5-turbo",  # Using OpenRouter model via environment config
    system_prompt="You are a coding expert. Provide clear, well-documented code solutions and explanations. Use the format_code_block tool to properly format code snippets. If you need specific information, use the fetch_qdrant_context tool to find relevant context.",
    tools=[format_code_block, fetch_qdrant_context, get_current_time]
)


# Create the Primary Agent (Orchestrator)
primary_agent = Agent(
    model="openai:gpt-3.5-turbo",  # Using OpenRouter model via environment config
    system_prompt="""
    You are a Primary Agent that orchestrates between a Research Agent and a Coding Agent.
    If the user asks a technical question, delegate to Coding Agent. If they ask about documentation or facts, delegate to Research Agent.
    Analyze the user's request and determine which agent is most appropriate:
    - If the request is about general information, research, or knowledge retrieval, use the research_agent.
    - If the request is about code, programming, or technical implementation, use the coding_agent.
    - Always provide helpful and accurate responses.
    """,
    tools=[fetch_qdrant_context, get_current_time]
)


@app.post("/chat")
async def chat_endpoint(request: ChatRequest, current_user: str = Depends(get_current_user)):
    if not openrouter_api_key:
        raise HTTPException(status_code=500, detail="Missing OPENROUTER_API_KEY environment variable")

    try:
        # Prepare the message for the primary agent
        full_message = f"User query: {request.message}"

        # Add conversation history if available
        if request.history:
            history_text = "\n".join([f"{msg.get('role', 'user')}: {msg.get('content', '')}" for msg in request.history])
            full_message = f"Conversation history:\n{history_text}\n\nCurrent query: {request.message}"

        # Run the primary agent to determine the appropriate sub-agent
        result = await primary_agent.run(full_message)

        # For this implementation, we'll use a simple approach to determine which agent to use
        # In a more complex system, you might want to parse the primary agent's output to decide
        user_message_lower = request.message.lower()

        # Determine which agent to use based on the query
        if any(keyword in user_message_lower for keyword in ["code", "program", "function", "python", "javascript", "java", "c++", "algorithm", "debug", "implement"]):
            # Use coding agent
            coding_result = await coding_agent.run(full_message)
            response = coding_result.data
        elif any(keyword in user_message_lower for keyword in ["research", "find", "information", "document", "knowledge", "learn", "explain"]):
            # Use research agent
            research_result = await research_agent.run(full_message)
            response = research_result.data
        else:
            # Default to primary agent's response
            response = result.data

        # Extract context sources if available in the result
        # For simplicity, we'll search the knowledge base again to get sources
        context_parts = []
        if qdrant_client:
            try:
                # Generate an embedding for the query text
                query_embedding = openai.embeddings.create(
                    model="text-embedding-ada-002",  # OpenAI embedding model via OpenRouter
                    input=request.message
                ).data[0].embedding
            except Exception as e:
                # Fallback to a simple embedding if OpenAI fails
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

            # Extract context sources
            context_sources = [r.payload.get("filename", "unknown") if hasattr(r, 'payload') and r.payload else "unknown" for r in search_results]
        else:
            context_sources = []

        return {"response": response, "context_sources": context_sources}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


# Alternative endpoint that allows specifying which agent to use directly
@app.post("/chat/research")
async def research_endpoint(request: ChatRequest, current_user: str = Depends(get_current_user)):
    """Direct endpoint for research-focused queries."""
    if not openrouter_api_key:
        raise HTTPException(status_code=500, detail="Missing OPENROUTER_API_KEY environment variable")

    try:
        # Prepare the message for the research agent
        full_message = f"User query: {request.message}"

        # Add conversation history if available
        if request.history:
            history_text = "\n".join([f"{msg.get('role', 'user')}: {msg.get('content', '')}" for msg in request.history])
            full_message = f"Conversation history:\n{history_text}\n\nCurrent query: {request.message}"

        # Run the research agent
        result = await research_agent.run(full_message)
        response = result.data

        # Extract context sources
        if qdrant_client:
            try:
                # Generate an embedding for the query text
                query_embedding = openai.embeddings.create(
                    model="text-embedding-ada-002",  # OpenAI embedding model via OpenRouter
                    input=request.message
                ).data[0].embedding
            except Exception as e:
                # Fallback to a simple embedding if OpenAI fails
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

            # Extract context sources
            context_sources = [r.payload.get("filename", "unknown") if hasattr(r, 'payload') and r.payload else "unknown" for r in search_results]
        else:
            context_sources = []

        return {"response": response, "context_sources": context_sources}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing research request: {str(e)}")


@app.post("/chat/coding")
async def coding_endpoint(request: ChatRequest, current_user: str = Depends(get_current_user)):
    """Direct endpoint for coding-focused queries."""
    if not openrouter_api_key:
        raise HTTPException(status_code=500, detail="Missing OPENROUTER_API_KEY environment variable")

    try:
        # Prepare the message for the coding agent
        full_message = f"User query: {request.message}"

        # Add conversation history if available
        if request.history:
            history_text = "\n".join([f"{msg.get('role', 'user')}: {msg.get('content', '')}" for msg in request.history])
            full_message = f"Conversation history:\n{history_text}\n\nCurrent query: {request.message}"

        # Run the coding agent
        result = await coding_agent.run(full_message)
        response = result.data

        # Extract context sources
        if qdrant_client:
            try:
                # Generate an embedding for the query text
                query_embedding = openai.embeddings.create(
                    model="text-embedding-ada-002",  # OpenAI embedding model via OpenRouter
                    input=request.message
                ).data[0].embedding
            except Exception as e:
                # Fallback to a simple embedding if OpenAI fails
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

            # Extract context sources
            context_sources = [r.payload.get("filename", "unknown") if hasattr(r, 'payload') and r.payload else "unknown" for r in search_results]
        else:
            context_sources = []

        return {"response": response, "context_sources": context_sources}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing coding request: {str(e)}")


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)