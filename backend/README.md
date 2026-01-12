# Robotics Textbook Backend

Backend API for the Physical AI & Humanoid Robotics textbook platform.

## Features

- **RAG Chatbot**: AI assistant powered by OpenAI and Qdrant vector database
- **Progress Tracking**: User progress and quiz management
- **Content Indexing**: Automatic embedding generation for textbook content

## Tech Stack

- **Framework**: FastAPI (Python 3.11+)
- **Database**: Neon Postgres (serverless PostgreSQL)
- **Vector DB**: Qdrant Cloud
- **AI/ML**: OpenAI API, LangChain

## Setup

### Prerequisites

- Python 3.11 or higher
- pip (Python package manager)
- PostgreSQL database (Neon recommended)
- Qdrant Cloud account
- OpenAI API key

### Installation

1. **Create virtual environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Configure environment**:
   ```bash
   cp .env.example .env
   # Edit .env with your credentials
   ```

4. **Initialize database**:
   ```bash
   python -m src.scripts.init_db
   ```

5. **Index content**:
   ```bash
   python -m src.scripts.index_content
   ```

## Running

### Development Server

```bash
uvicorn src.main:app --reload --port 8000
```

API will be available at: http://localhost:8000

API Documentation: http://localhost:8000/v1/docs

### Production

```bash
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## API Endpoints

### Chatbot API

- `POST /v1/chat` - Send message to AI assistant
- `GET /v1/chat/history` - Get conversation history
- `POST /v1/chat/context` - Set chapter context

### Progress API

- `GET /v1/progress` - Get user progress summary
- `POST /v1/progress/chapter` - Update chapter progress
- `POST /v1/quiz/submit` - Submit quiz answers
- `GET /v1/quiz/attempts` - Get quiz attempts
- `GET /v1/stats` - Get learning statistics

## Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app entry point
│   ├── config.py            # Configuration management
│   ├── models/              # Data models
│   │   ├── user.py
│   │   ├── conversation.py
│   │   ├── progress.py
│   │   ├── quiz.py
│   │   └── embedding.py
│   ├── services/            # Business logic
│   │   ├── database.py
│   │   ├── chatbot_service.py
│   │   ├── vector_service.py
│   │   ├── progress_service.py
│   │   ├── quiz_service.py
│   │   └── content_indexer.py
│   ├── api/                 # API endpoints
│   │   ├── chatbot.py
│   │   └── progress.py
│   └── scripts/             # Utility scripts
│       ├── init_db.py
│       └── index_content.py
├── tests/                   # Test files
├── requirements.txt         # Python dependencies
├── .env.example            # Environment template
└── README.md               # This file
```

## Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src tests/

# Run specific test file
pytest tests/test_chatbot.py
```

## Deployment

### Docker

```bash
# Build image
docker build -t robotics-textbook-api .

# Run container
docker run -p 8000:8000 --env-file .env robotics-textbook-api
```

### Vercel

```bash
# Install Vercel CLI
npm install -g vercel

# Deploy
vercel
```

## Environment Variables

See `.env.example` for all required environment variables.

### Required Variables

- `OPENAI_API_KEY` - OpenAI API key
- `DATABASE_URL` - PostgreSQL connection string
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `SECRET_KEY` - Application secret key

## Development

### Code Style

```bash
# Format code
black src/

# Lint code
flake8 src/

# Type checking
mypy src/
```

### Adding New Endpoints

1. Create model in `src/models/`
2. Implement service logic in `src/services/`
3. Create API endpoint in `src/api/`
4. Add router to `src/main.py`
5. Write tests in `tests/`

## Troubleshooting

### Database Connection Issues

- Verify `DATABASE_URL` in `.env`
- Check Neon Postgres instance is running
- Test connection: `psql $DATABASE_URL`

### Qdrant Connection Issues

- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check Qdrant cluster is active
- Test connection in Python:
  ```python
  from qdrant_client import QdrantClient
  client = QdrantClient(url="your-url", api_key="your-key")
  print(client.get_collections())
  ```

### OpenAI API Issues

- Verify `OPENAI_API_KEY` is valid
- Check API quota and billing
- Monitor rate limits

## License

Copyright © 2026 Physical AI & Humanoid Robotics textbook

## Support

- GitHub Issues: https://github.com/aurangzaibmughal/Ai--book-/issues
- Documentation: https://ai-book-five-mauve.vercel.app
