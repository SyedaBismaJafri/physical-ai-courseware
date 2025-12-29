import asyncio
import logging
from typing import List, Dict, Optional, Tuple
import os
from pathlib import Path
import aiofiles
import markdown
from bs4 import BeautifulSoup
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import openai
from pydantic import BaseModel
import hashlib
import time

# Set up logging
logger = logging.getLogger(__name__)

class DocumentChunk(BaseModel):
    id: str
    content: str
    source_file: str
    section_title: str
    url_path: str
    metadata: Dict

class RAGService:
    def __init__(self, qdrant_url: str, qdrant_api_key: Optional[str], collection_name: str, openrouter_api_key: str):
        self.qdrant_client = AsyncQdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.collection_name = collection_name
        self.openrouter_api_key = openrouter_api_key
        self.openai_client = openai.AsyncOpenAI(
            api_key=openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Initialize the collection if it doesn't exist
        asyncio.create_task(self._init_collection())

    async def _init_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        try:
            collections = await self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                await self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),  # Qwen embeddings size
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {e}")
            raise

    async def _get_embedding(self, text: str) -> List[float]:
        """Get embedding for text using OpenRouter Qwen model"""
        try:
            response = await self.openai_client.embeddings.create(
                model="nomic-ai/nomic-embed-text-v1.5",  # Using a compatible embedding model
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error getting embedding: {e}")
            raise

    async def _chunk_document(self, content: str, source_file: str, url_path: str, max_length: int = 512, overlap: int = 50) -> List[DocumentChunk]:
        """Split document content into chunks"""
        chunks = []

        # Simple sentence-based chunking
        sentences = content.split('. ')
        current_chunk = ""
        chunk_num = 0

        for sentence in sentences:
            # Add period back to sentence (except for the last one)
            sentence = sentence.strip()
            if sentence and not sentence.endswith('.'):
                sentence += '.'

            if len(current_chunk) + len(sentence) < max_length:
                current_chunk += " " + sentence if current_chunk else sentence
            else:
                if current_chunk.strip():
                    chunk_id = hashlib.md5(f"{source_file}_{chunk_num}".encode()).hexdigest()
                    chunks.append(DocumentChunk(
                        id=chunk_id,
                        content=current_chunk.strip(),
                        source_file=source_file,
                        section_title=f"Section {chunk_num}",
                        url_path=url_path,
                        metadata={"chunk_num": chunk_num}
                    ))
                    chunk_num += 1

                # Start a new chunk, potentially with overlap
                current_chunk = sentence
                # Add overlap if possible
                if len(current_chunk) < overlap:
                    continue

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunk_id = hashlib.md5(f"{source_file}_{chunk_num}".encode()).hexdigest()
            chunks.append(DocumentChunk(
                id=chunk_id,
                content=current_chunk.strip(),
                source_file=source_file,
                section_title=f"Section {chunk_num}",
                url_path=url_path,
                metadata={"chunk_num": chunk_num}
            ))

        return chunks

    async def _extract_markdown_content(self, file_path: str) -> Tuple[str, str]:
        """Extract content and title from markdown file"""
        async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
            content = await f.read()

        # Extract title from frontmatter or first heading
        title = "Untitled"
        lines = content.split('\n')

        # Look for frontmatter
        if lines and lines[0].strip() == '---':
            for i, line in enumerate(lines[1:], 1):
                if line.strip() == '---':
                    # Found end of frontmatter, look for title
                    for frontmatter_line in lines[1:i]:
                        if frontmatter_line.startswith('title:'):
                            title = frontmatter_line.split(':', 1)[1].strip().strip('"\'')
                            break
                    break

        # If no title found in frontmatter, look for first heading
        if title == "Untitled":
            for line in lines:
                if line.startswith('# '):
                    title = line[2:].strip()
                    break

        # Convert markdown to plain text for indexing
        html = markdown.markdown(content)
        soup = BeautifulSoup(html, 'html.parser')
        plain_text = soup.get_text()

        # Create URL path from file path
        path = Path(file_path)
        docs_dir = Path(file_path).parent  # This should be adjusted based on actual docs location

        # Generate URL path from file structure
        relative_path = path.relative_to(path.parent.parent)  # Adjust based on actual structure
        url_path = f"/docs/{relative_path.with_suffix('').as_posix()}"

        return plain_text, title, url_path

    async def ingest_documents(self, docs_path: str) -> Dict:
        """Ingest documents from the specified path"""
        start_time = time.time()
        documents_processed = 0
        chunks_created = 0

        try:
            # Find all markdown files
            docs_dir = Path(docs_path)
            markdown_files = list(docs_dir.rglob("*.md")) + list(docs_dir.rglob("*.mdx"))

            all_chunks = []

            for md_file in markdown_files:
                try:
                    content, title, url_path = await self._extract_markdown_content(str(md_file))

                    # Create chunks from the document
                    chunks = await self._chunk_document(content, str(md_file), url_path)
                    all_chunks.extend(chunks)

                    documents_processed += 1
                    logger.info(f"Processed document: {md_file}")
                except Exception as e:
                    logger.error(f"Error processing document {md_file}: {e}")
                    continue

            # Upload chunks to Qdrant
            if all_chunks:
                points = []
                for chunk in all_chunks:
                    # Get embedding for the chunk
                    vector = await self._get_embedding(chunk.content)

                    point = models.PointStruct(
                        id=chunk.id,
                        vector=vector,
                        payload={
                            "content": chunk.content,
                            "source_file": chunk.source_file,
                            "section_title": chunk.section_title,
                            "url_path": chunk.url_path,
                            "metadata": chunk.metadata
                        }
                    )
                    points.append(point)

                # Upload in batches
                batch_size = 100
                for i in range(0, len(points), batch_size):
                    batch = points[i:i+batch_size]
                    await self.qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=batch
                    )
                    logger.info(f"Uploaded batch of {len(batch)} points to Qdrant")

                chunks_created = len(all_chunks)

            processing_time = time.time() - start_time

            return {
                "status": "completed",
                "documents_processed": documents_processed,
                "chunks_created": chunks_created,
                "processing_time": processing_time
            }

        except Exception as e:
            logger.error(f"Error during document ingestion: {e}")
            raise

    async def query_documents(self, query: str, selected_text: Optional[str] = None, top_k: int = 5) -> List[Dict]:
        """Query documents using semantic search"""
        try:
            # Combine query and selected text if provided
            search_text = query
            if selected_text:
                search_text = f"{query} Context: {selected_text}"

            # Get embedding for the query
            query_embedding = await self._get_embedding(search_text)

            # Perform semantic search in Qdrant
            search_results = await self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            results = []
            for hit in search_results:
                if hit.payload:
                    results.append({
                        "content": hit.payload.get("content", ""),
                        "source_file": hit.payload.get("source_file", ""),
                        "section_title": hit.payload.get("section_title", ""),
                        "url_path": hit.payload.get("url_path", ""),
                        "relevance_score": hit.score
                    })

            return results

        except Exception as e:
            logger.error(f"Error during document query: {e}")
            raise

    async def generate_response(self, query: str, context_chunks: List[Dict], selected_text: Optional[str] = None) -> str:
        """Generate response using LLM with context"""
        try:
            # Build context from retrieved chunks
            context = "\n\n".join([chunk["content"] for chunk in context_chunks])

            # Build the prompt
            prompt = f"""
            You are an AI assistant for the Physical AI & Humanoid Robotics course.
            Answer the user's question based on the provided context from the course documentation.
            If the context doesn't contain the information needed to answer the question, say so.

            Context from documentation:
            {context}

            User's question: {query}
            """

            if selected_text:
                prompt += f"\nUser also selected this text: {selected_text}"

            # Call the LLM
            response = await self.openai_client.chat.completions.create(
                model="openchat/openchat-7b:free",  # Using a free model from OpenRouter
                messages=[{"role": "user", "content": prompt}],
                max_tokens=500,
                temperature=0.7
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            return "Sorry, I encountered an error while generating a response. Please try again."