# RAG-Enabled Book - Requirements Specification

## 1. Overview

This document specifies the requirements for implementing a Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics courseware. The system will provide an AI-powered chatbot that can answer questions based on the course documentation.

## 2. Frontend Requirements

### 2.1 Floating React Chatbot Widget
- **Location**: Integrated into the Docusaurus documentation site
- **Position**: Fixed floating widget in the bottom-right corner of the screen
- **Size**: Responsive design that works on all screen sizes
- **Toggle**: Click to expand/collapse the chat interface
- **Styling**: Consistent with the Docusaurus theme and courseware branding

### 2.2 Selection-Aware Queries
- **Feature**: Use `window.getSelection()` to capture selected text
- **Context**: When user selects text and interacts with chatbot, include selection in query
- **Highlight**: Selected text should be visually highlighted when sent to chatbot
- **Fallback**: If no text selected, allow standard text input

### 2.3 Widget Components
- **Input Area**: Text input field for questions
- **Message History**: Display conversation history within the widget
- **Loading State**: Show loading indicators during API calls
- **Error Handling**: Display user-friendly error messages
- **Clear Chat**: Option to clear the current conversation

### 2.4 User Experience
- **Accessibility**: WCAG 2.1 AA compliance for keyboard navigation and screen readers
- **Performance**: Load in < 200ms, respond to queries in < 5s
- **Mobile Support**: Responsive design for mobile devices
- **Persistence**: Maintain conversation state across page navigations

## 3. Backend Requirements

### 3.1 RAG Service Structure
Location: `/backend/rag_service/`

### 3.2 API Server
- **Framework**: FastAPI server
- **Endpoints**:
  - `POST /query` - Process user queries and return RAG responses
  - `POST /ingest` - Load and process documentation files
- **Documentation**: Auto-generated API documentation with Swagger UI
- **Rate Limiting**: Implement rate limiting to prevent abuse

### 3.3 Service Dependencies
- **Python Version**: 3.9+ compatibility
- **Async Support**: Use async/await for I/O operations
- **Logging**: Comprehensive logging for debugging and monitoring
- **Configuration**: Environment-based configuration management

## 4. API Endpoints Specification

### 4.1 Query Endpoint
```
POST /query
```

**Request Body**:
```json
{
  "query": "string",
  "selected_text": "string (optional)",
  "session_id": "string (optional)",
  "context": {
    "current_page": "string (URL or path)",
    "course_module": "string"
  }
}
```

**Response**:
```json
{
  "response": "string",
  "sources": [
    {
      "title": "string",
      "url": "string",
      "relevance_score": "number"
    }
  ],
  "session_id": "string",
  "timestamp": "ISO string"
}
```

### 4.2 Ingestion Endpoint
```
POST /ingest
```

**Request Body**:
```json
{
  "source_path": "string (path to docs directory)",
  "force_rebuild": "boolean (optional, default: false)"
}
```

**Response**:
```json
{
  "status": "string",
  "documents_processed": "number",
  "chunks_created": "number",
  "processing_time": "number (seconds)"
}
```

## 5. Embeddings Requirements

### 5.1 Model Selection
- **Provider**: OpenRouter
- **Model**: Qwen embedding models
- **Configuration**: Environment variables for API key and model selection

### 5.2 Embedding Process
- **Chunking Strategy**: Recursive character text splitter (max 512 tokens)
- **Overlap**: 50-token overlap between chunks to preserve context
- **Metadata**: Include source file path, section title, and page number
- **Batch Processing**: Process multiple documents in parallel

### 5.3 Quality Requirements
- **Dimensionality**: Consistent embedding dimensions across all chunks
- **Normalization**: Normalize embeddings for cosine similarity
- **Deduplication**: Remove duplicate or highly similar chunks

## 6. Vector Store Configuration

### 6.1 Qdrant Cloud Setup
- **Tier**: Free tier with sufficient storage for course documentation
- **Collection**: Dedicated collection for courseware documents
- **Configuration**: Cosine similarity search, HNSW indexing
- **Connection**: Secure API key authentication

### 6.2 Collection Schema
```
Collection: "physical_ai_docs"
- vector: float vector (embedding dimensions)
- payload: {
    "doc_id": "string",
    "content": "string",
    "source_file": "string",
    "section_title": "string",
    "url_path": "string",
    "metadata": "object"
  }
```

### 6.3 Search Configuration
- **Similarity**: Cosine similarity for semantic search
- **Results**: Return top 5 most relevant chunks
- **Threshold**: Minimum relevance score threshold (0.7)
- **Filtering**: Filter results by course module if specified

## 7. Chat History Storage

### 7.1 Neon Serverless Postgres
- **Tier**: Serverless Postgres for automatic scaling
- **Connection**: Secure connection with connection pooling
- **Schema**: Optimized for chat session storage and retrieval

### 7.2 Database Schema
```sql
-- Chat sessions table
CREATE TABLE chat_sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  metadata JSONB
);

-- Chat messages table
CREATE TABLE chat_messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_id UUID REFERENCES chat_sessions(id),
  role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
  content TEXT NOT NULL,
  timestamp TIMESTAMP DEFAULT NOW(),
  metadata JSONB
);
```

### 7.3 Session Management
- **Persistence**: Store conversation history across user sessions
- **Cleanup**: Automatic cleanup of old sessions (after 30 days)
- **Privacy**: GDPR-compliant data handling and deletion

## 8. Data Source Integration

### 8.1 Documentation Source
- **Location**: `/frontend/docs/` directory
- **Format**: Markdown files with frontmatter
- **Structure**: Follow existing directory structure (overview, module1-ros2, etc.)

### 8.2 Content Processing
- **Parsing**: Extract text content while preserving structure
- **Frontmatter**: Extract and store frontmatter metadata
- **Links**: Preserve and index internal documentation links
- **Images**: Extract alt text and captions for indexing

### 8.3 Update Mechanism
- **Real-time Sync**: Detect changes in docs directory and update vector store
- **Incremental Updates**: Only process changed/added files
- **Validation**: Ensure all referenced files exist before ingestion

## 9. Security Requirements

### 9.1 API Security
- **Authentication**: Optional API key for external access
- **Rate Limiting**: Prevent API abuse and excessive usage
- **Input Validation**: Sanitize all user inputs to prevent injection

### 9.2 Data Privacy
- **Data Isolation**: Separate user data appropriately
- **Anonymization**: Do not store personally identifiable information
- **Compliance**: Follow applicable privacy regulations

## 10. Performance Requirements

### 10.1 Response Times
- **Query Response**: < 5 seconds for typical queries
- **Ingestion**: Process 100 pages in < 30 seconds
- **Initial Load**: Chat widget loads in < 200ms

### 10.2 Scalability
- **Concurrent Users**: Support 50+ concurrent users
- **Document Scale**: Handle 1000+ documentation pages
- **Memory Usage**: Optimize for minimal memory footprint

## 11. Deployment Requirements

### 11.1 Backend Deployment
- **Containerization**: Docker container for easy deployment
- **Environment**: Support for cloud deployment (AWS, GCP, Azure)
- **Monitoring**: Health checks and monitoring endpoints

### 11.2 Frontend Integration
- **Build Process**: Minimal impact on Docusaurus build time
- **CDN**: Serve widget assets via CDN for optimal performance
- **Falling Back**: Graceful degradation if RAG service is unavailable

## 12. Testing Requirements

### 12.1 Unit Tests
- **Backend**: Test all API endpoints and business logic
- **Frontend**: Test React components and state management
- **Coverage**: >80% code coverage

### 12.2 Integration Tests
- **API**: Test end-to-end RAG functionality
- **UI**: Test chatbot widget functionality
- **Performance**: Load testing for concurrent users

## 13. Monitoring and Observability

### 13.1 Logging
- **Structured Logging**: JSON-formatted logs with context
- **Error Tracking**: Capture and report errors for debugging
- **Usage Analytics**: Track query patterns and usage

### 13.2 Metrics
- **Response Times**: Monitor API response times
- **Success Rates**: Track query success/error rates
- **Usage**: Monitor number of queries and active sessions