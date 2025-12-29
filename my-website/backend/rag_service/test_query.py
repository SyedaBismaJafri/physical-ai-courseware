from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_url and qdrant_api_key:
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
    )

    # Test different query methods
    print("Testing different query approaches...")

    try:
        # Try query_points with text query
        print("\n1. Testing query_points with text...")
        results = qdrant_client.query_points(
            collection_name="book_docs",
            query="Hello",
            limit=2
        )
        print(f"Success! Found {len(results.points)} results")
    except Exception as e:
        print(f"Error with query_points: {e}")

    try:
        # Try search with query parameter instead of query_text
        print("\n2. Testing search method...")
        results = qdrant_client.search(
            collection_name="book_docs",
            query="Hello",
            limit=2
        )
        print(f"Success! Found {len(results)} results")
    except Exception as e:
        print(f"Error with search: {e}")

    try:
        # Try query with query_filter for full text search
        print("\n3. Testing query with filter...")
        results = qdrant_client.query_points(
            collection_name="book_docs",
            query="Hello",
            query_filter=None,
            limit=2
        )
        print(f"Success! Found {len(results.points)} results")
    except Exception as e:
        print(f"Error with query with filter: {e}")

    try:
        # Try scroll for general browsing
        print("\n4. Testing scroll method to see if collection has data...")
        results = qdrant_client.scroll(
            collection_name="book_docs",
            limit=2
        )
        print(f"Scroll success! Found {len(results[0])} results out of total collection")
    except Exception as e:
        print(f"Error with scroll: {e}")