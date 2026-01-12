# Developer Quickstart Guide

**Feature**: Physical AI & Humanoid Robotics Textbook Platform
**Date**: 2026-01-11
**Audience**: Developers contributing to the platform

## Overview

This guide helps you set up a local development environment for the Physical AI & Humanoid Robotics textbook platform. The platform consists of:
- **Frontend**: Docusaurus static site (docs, UI components)
- **Backend**: FastAPI RAG chatbot API (optional for content-only development)

## Prerequisites

### Required Software

- **Node.js** 18.0 or higher ([Download](https://nodejs.org/))
- **npm** 9.0 or higher (comes with Node.js)
- **Git** 2.0 or higher ([Download](https://git-scm.com/))

### Optional (for backend development)

- **Python** 3.11 or higher ([Download](https://www.python.org/))
- **pip** (comes with Python)
- **PostgreSQL** client tools (for database access)

### Recommended Tools

- **VS Code** with extensions:
  - ESLint
  - Prettier
  - Python (if working on backend)
  - Markdown All in One
- **Docker Desktop** (for backend containerization)

## Quick Start (Frontend Only)

If you're only working on content or frontend components, follow these steps:

### 1. Clone Repository

```bash
git clone https://github.com/your-username/ai-book.git
cd ai-book
```

### 2. Install Dependencies

```bash
npm install
```

If you encounter peer dependency issues:

```bash
npm install --legacy-peer-deps
```

### 3. Start Development Server

```bash
npm start
```

The site will open at [http://localhost:3000](http://localhost:3000) with hot reload enabled.

### 4. Make Changes

- **Content**: Edit Markdown files in `docs/chapters/`
- **Components**: Edit React components in `src/components/`
- **Styles**: Edit CSS in `src/css/custom.css`
- **Configuration**: Edit `docusaurus.config.js` or `sidebars.js`

### 5. Build for Production

```bash
npm run build
```

Built files will be in `build/` directory.

### 6. Test Production Build Locally

```bash
npm run serve
```

## Full Setup (Frontend + Backend)

For complete platform development including the RAG chatbot:

### 1. Clone and Install Frontend

Follow steps 1-2 from "Quick Start" above.

### 2. Set Up Backend

#### Install Python Dependencies

```bash
cd backend
pip install -r requirements.txt
```

Or use a virtual environment (recommended):

```bash
cd backend
python -m venv venv

# On Windows
venv\Scripts\activate

# On macOS/Linux
source venv/bin/activate

pip install -r requirements.txt
```

#### Configure Environment Variables

Create `.env` file in `backend/` directory:

```bash
# Copy example environment file
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# OpenAI API
OPENAI_API_KEY=sk-your-api-key-here

# Neon Postgres
DATABASE_URL=postgresql://user:password@host/database

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000
```

**Getting API Keys:**
- **OpenAI**: [platform.openai.com/api-keys](https://platform.openai.com/api-keys)
- **Neon Postgres**: [console.neon.tech](https://console.neon.tech) (free tier available)
- **Qdrant Cloud**: [cloud.qdrant.io](https://cloud.qdrant.io) (free tier available)

### 3. Initialize Database

```bash
cd backend
python -m src.scripts.init_db
```

This creates necessary tables in Neon Postgres.

### 4. Index Content for RAG

```bash
cd backend
python -m src.scripts.index_content
```

This generates embeddings for all textbook content and stores them in Qdrant.

### 5. Start Backend Server

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

Backend API will be available at [http://localhost:8000](http://localhost:8000)

API documentation: [http://localhost:8000/docs](http://localhost:8000/docs)

### 6. Start Frontend (in separate terminal)

```bash
# From repo root
npm start
```

Frontend will be at [http://localhost:3000](http://localhost:3000) and will connect to backend at `http://localhost:8000`.

## Project Structure

```
ai-book/
├── docs/                          # Markdown content
│   ├── intro.md
│   └── chapters/
│       ├── chapter-01-introduction/
│       │   ├── index.md
│       │   ├── learning-objectives.md
│       │   ├── concepts.md
│       │   ├── hands-on-lab.md
│       │   ├── exercises.md
│       │   ├── quiz.md
│       │   └── faqs.md
│       └── ... (chapters 02-13)
├── src/                           # React components
│   ├── components/
│   │   ├── ChatbotWidget/
│   │   ├── QuizComponent/
│   │   └── ProgressTracker/
│   ├── css/
│   │   └── custom.css
│   └── pages/
├── static/                        # Static assets
│   └── img/
├── backend/                       # FastAPI backend
│   ├── src/
│   │   ├── main.py
│   │   ├── models/
│   │   ├── services/
│   │   ├── api/
│   │   └── scripts/
│   ├── tests/
│   ├── requirements.txt
│   └── .env
├── docusaurus.config.js           # Docusaurus configuration
├── sidebars.js                    # Navigation structure
├── package.json                   # Frontend dependencies
└── README.md
```

## Common Tasks

### Adding a New Chapter

1. Create chapter directory:
   ```bash
   mkdir -p docs/chapters/chapter-XX-topic-name
   ```

2. Create required files (use existing chapter as template):
   ```bash
   cd docs/chapters/chapter-XX-topic-name
   touch index.md learning-objectives.md concepts.md hands-on-lab.md exercises.md quiz.md faqs.md
   ```

3. Update `sidebars.js`:
   ```javascript
   {
     type: 'category',
     label: 'Chapter XX: Topic Name',
     items: [
       'chapters/chapter-XX-topic-name/index',
       'chapters/chapter-XX-topic-name/learning-objectives',
       // ... other sections
     ],
   }
   ```

4. Write content following the 12-section template

5. Re-index content for RAG (if backend is set up):
   ```bash
   cd backend
   python -m src.scripts.index_content
   ```

### Adding a Custom Component

1. Create component directory:
   ```bash
   mkdir src/components/MyComponent
   ```

2. Create component files:
   ```bash
   cd src/components/MyComponent
   touch index.tsx MyComponent.module.css
   ```

3. Implement component in `index.tsx`

4. Import and use in MDX files:
   ```mdx
   import MyComponent from '@site/src/components/MyComponent';

   <MyComponent prop="value" />
   ```

### Running Tests

**Frontend tests:**
```bash
npm test
```

**Backend tests:**
```bash
cd backend
pytest
```

**E2E tests:**
```bash
npm run test:e2e
```

### Validating Content Structure

Ensure all chapters have 12 mandatory sections:

```bash
npm run validate:content
```

### Building for Production

**Frontend:**
```bash
npm run build
```

**Backend (Docker):**
```bash
cd backend
docker build -t robotics-textbook-api .
docker run -p 8000:8000 --env-file .env robotics-textbook-api
```

## Development Workflow

### Branch Strategy

- `main` - Production-ready code
- `001-robotics-textbook` - Feature branch for textbook platform
- Create feature branches from `001-robotics-textbook` for specific work

### Making Changes

1. Create feature branch:
   ```bash
   git checkout -b feature/my-feature
   ```

2. Make changes and test locally

3. Commit with descriptive message:
   ```bash
   git add .
   git commit -m "Add quiz component for Chapter 1"
   ```

4. Push and create pull request:
   ```bash
   git push origin feature/my-feature
   ```

### Code Style

**Frontend (JavaScript/TypeScript):**
- Use Prettier for formatting
- Follow ESLint rules
- Use TypeScript for type safety

**Backend (Python):**
- Follow PEP 8 style guide
- Use type hints
- Format with Black

**Markdown:**
- Use consistent heading levels
- Include alt text for images
- Follow template structure for chapters

## Troubleshooting

### Port Already in Use

If port 3000 or 8000 is already in use:

```bash
# Frontend - use different port
npm start -- --port 3001

# Backend - use different port
uvicorn src.main:app --reload --port 8001
```

### Module Not Found Errors

Clear cache and reinstall:

```bash
# Frontend
rm -rf node_modules package-lock.json
npm install

# Backend
pip uninstall -r requirements.txt -y
pip install -r requirements.txt
```

### Database Connection Issues

Verify DATABASE_URL in `.env`:
- Check username, password, host, database name
- Ensure Neon Postgres instance is running
- Test connection: `psql $DATABASE_URL`

### Qdrant Connection Issues

Verify Qdrant credentials:
- Check QDRANT_URL and QDRANT_API_KEY in `.env`
- Test connection in Python:
  ```python
  from qdrant_client import QdrantClient
  client = QdrantClient(url="your-url", api_key="your-key")
  print(client.get_collections())
  ```

### Build Failures

Check for:
- Syntax errors in Markdown files
- Missing images referenced in content
- Invalid frontmatter in MDX files
- Broken internal links

Run validation:
```bash
npm run validate:content
npm run validate:links
```

## Performance Optimization

### Frontend

- Optimize images: Use WebP format, compress with `npm run optimize:images`
- Lazy-load components: Use React.lazy() for heavy components
- Code splitting: Docusaurus handles automatically

### Backend

- Enable caching: Redis for frequently accessed data
- Optimize queries: Use database indexes, limit result sets
- Rate limiting: Prevent API abuse

## Deployment

### Frontend (GitHub Pages)

Automatic deployment on push to `main`:

```bash
git push origin main
```

GitHub Actions workflow deploys to GitHub Pages.

### Backend (Vercel)

1. Install Vercel CLI:
   ```bash
   npm install -g vercel
   ```

2. Deploy:
   ```bash
   cd backend
   vercel
   ```

3. Set environment variables in Vercel dashboard

## Getting Help

- **Documentation**: [Docusaurus Docs](https://docusaurus.io/docs)
- **Issues**: [GitHub Issues](https://github.com/your-username/ai-book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/ai-book/discussions)

## Next Steps

1. ✅ Development environment set up
2. ➡️ Review existing chapters for content structure
3. ➡️ Familiarize yourself with component library
4. ➡️ Check open issues for contribution opportunities
5. ➡️ Read [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines

## Useful Commands Reference

```bash
# Frontend
npm start              # Start dev server
npm run build          # Build for production
npm run serve          # Serve production build
npm test               # Run tests
npm run clear          # Clear cache

# Backend
uvicorn src.main:app --reload    # Start dev server
pytest                           # Run tests
python -m src.scripts.init_db    # Initialize database
python -m src.scripts.index_content  # Index content for RAG

# Git
git status             # Check status
git branch             # List branches
git checkout -b name   # Create new branch
git add .              # Stage changes
git commit -m "msg"    # Commit changes
git push origin branch # Push to remote
```

## Environment Variables Reference

### Frontend (.env)

```env
# Optional - only needed if customizing API endpoints
REACT_APP_API_URL=http://localhost:8000
```

### Backend (.env)

```env
# Required
OPENAI_API_KEY=sk-...
DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...

# Optional
ENVIRONMENT=development
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000
MAX_CONVERSATION_LENGTH=50
EMBEDDING_MODEL=text-embedding-ada-002
CHAT_MODEL=gpt-4
```

---

**Last Updated**: 2026-01-11
**Maintainer**: Development Team
