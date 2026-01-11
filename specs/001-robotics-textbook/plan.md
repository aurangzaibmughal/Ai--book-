# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-robotics-textbook` | **Date**: 2026-01-11 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-robotics-textbook/spec.md`

## Summary

Build an AI-native educational platform delivering a 13-week Physical AI & Humanoid Robotics curriculum through a Docusaurus-based static site with embedded RAG chatbot. Platform serves 13 chapters across 4 modules (ROS 2, Simulation, NVIDIA Isaac, Vision-Language-Action) with mandatory 12-section structure per chapter, hands-on labs with 3 hardware tiers, interactive exercises, quizzes, and contextual AI assistance. Deployed to GitHub Pages for public access with <2s page load, <1s search, and 99%+ uptime targets.

## Technical Context

**Language/Version**:
- Frontend: JavaScript/TypeScript (Node.js 18+, React 18+ via Docusaurus 3.x)
- Backend: Python 3.11+ (FastAPI 0.104+)
- Content: Markdown with MDX support

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, @docusaurus/preset-classic, docusaurus-search-local, mermaid (diagrams)
- Backend: FastAPI, OpenAI Python SDK (ChatKit), langchain, qdrant-client, psycopg2 (Neon Postgres)
- Deployment: GitHub Actions, gh-pages

**Storage**:
- Content: Git repository (Markdown files)
- User Progress: Neon Postgres (serverless PostgreSQL)
- Vector Embeddings: Qdrant Cloud (managed vector database)
- Conversation History: Neon Postgres

**Testing**:
- Frontend: Jest + React Testing Library (component tests)
- Backend: pytest + httpx (API tests)
- E2E: Playwright (critical user flows)
- Content: Custom validators (12-section structure, code executability)

**Target Platform**:
- Frontend: Static site hosted on GitHub Pages (CDN-distributed)
- Backend: Cloud-hosted API (Vercel/Railway/Render for FastAPI)
- Browsers: Chrome, Firefox, Safari, Edge (last 2 versions)
- Mobile: Responsive web (375px+ width)

**Project Type**: Web application (frontend static site + backend API)

**Performance Goals**:
- Page load: <2 seconds (p95) on 10 Mbps+ connections
- Search: <1 second response time (95% of queries)
- AI Assistant: <3 seconds response time (95% of queries)
- Concurrent users: 1,000 without degradation
- Uptime: 99%+ over 30-day periods

**Constraints**:
- GitHub Pages: Static files only, no server-side rendering
- Content size: Optimize images/diagrams for fast loading
- API rate limits: OpenAI API quotas for chatbot
- Free tier limits: Neon Postgres (1GB), Qdrant Cloud (1GB vectors)
- Build time: <10 minutes for full site rebuild

**Scale/Scope**:
- Content: 13 chapters × 12 sections = 156 content pages
- Code examples: ~50-100 executable code snippets
- Diagrams: ~30-50 Mermaid diagrams
- Quizzes: 13 quizzes × 7 questions avg = ~91 questions
- Expected users: 1,000-10,000 students over first year
- Languages: English (primary), Urdu (optional Phase 2)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Technical Accuracy & Educational Rigor
- ✅ **PASS**: Code examples will be validated on Ubuntu 22.04 LTS before publication
- ✅ **PASS**: ROS 2 Humble/Iron compatibility enforced through testing
- ✅ **PASS**: Hardware tiers (min/recommended/premium) documented per chapter
- ✅ **PASS**: Mermaid diagrams for visual concepts, hands-on labs for practical skills
- ✅ **PASS**: Content review process includes SME validation

### Principle II: Structured Learning Progression
- ✅ **PASS**: Docusaurus template enforces 12-section structure per chapter
- ✅ **PASS**: Custom linter validates all mandatory sections present
- ✅ **PASS**: Consistent navigation and layout across all chapters

### Principle III: Beginner-Friendly Progression (NON-NEGOTIABLE)
- ✅ **PASS**: Module structure enforced: ROS 2 → Simulation → Isaac → VLA
- ✅ **PASS**: Prerequisites explicitly listed in each chapter frontmatter
- ✅ **PASS**: Sidebar navigation shows prerequisite chains visually

### Principle IV: Hands-On Lab Quality
- ✅ **PASS**: Lab template includes: instructions, dependencies, expected output, troubleshooting
- ✅ **PASS**: CI/CD validates code examples execute successfully
- ✅ **PASS**: Comments explain robotics concepts, not just code mechanics

### Principle V: Deployment & Accessibility
- ✅ **PASS**: GitHub Pages provides 99%+ uptime (GitHub SLA)
- ✅ **PASS**: Docusaurus optimized for <2s page load (code splitting, prefetching)
- ✅ **PASS**: Responsive design tested at 375px+ width
- ✅ **PASS**: docusaurus-search-local provides <1s search
- ✅ **PASS**: i18n plugin supports English + Urdu

### Principle VI: RAG Chatbot Integration
- ✅ **PASS**: FastAPI backend with OpenAI SDK for content-grounded responses
- ✅ **PASS**: Qdrant vector DB for semantic search over textbook content
- ✅ **PASS**: Neon Postgres for conversation history persistence
- ✅ **PASS**: <3s response time achievable with streaming responses
- ✅ **PASS**: Graceful degradation with fallback to search

### Principle VII: Code Standards & Compatibility
- ✅ **PASS**: All code examples target Ubuntu 22.04 LTS
- ✅ **PASS**: ROS 2 version (Humble/Iron) documented per example
- ✅ **PASS**: Error handling in all code examples
- ✅ **PASS**: Inline comments explain concepts
- ✅ **PASS**: CI/CD runs code validation tests

**Constitution Compliance**: ✅ ALL GATES PASSED - No violations, proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-robotics-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - technology decisions
├── data-model.md        # Phase 1 output - entities and schemas
├── quickstart.md        # Phase 1 output - developer setup guide
├── contracts/           # Phase 1 output - API specifications
│   ├── chatbot-api.yaml # OpenAPI spec for RAG chatbot
│   └── progress-api.yaml # OpenAPI spec for progress tracking
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus static site)
docs/
├── intro.md                          # Homepage
└── chapters/                         # Chapter content
    ├── chapter-01-introduction/
    │   ├── index.md                  # Overview
    │   ├── learning-objectives.md
    │   ├── concepts.md
    │   ├── hands-on-lab.md
    │   ├── exercises.md
    │   ├── quiz.md
    │   └── faqs.md
    ├── chapter-02-ros2-architecture/
    └── ... (chapters 03-13)

src/
├── components/                       # Custom React components
│   ├── ChatbotWidget/               # Embedded AI assistant
│   ├── QuizComponent/               # Interactive quizzes
│   ├── ProgressTracker/             # Chapter completion tracking
│   └── HardwareTierSelector/        # Hardware tier display
├── css/
│   └── custom.css                   # Theme customization
└── pages/                           # Custom pages
    └── progress.tsx                 # Progress dashboard

static/
├── img/                             # Images and diagrams
└── fonts/                           # Custom fonts (if needed)

i18n/
└── ur/                              # Urdu translations (Phase 2)
    └── docusaurus-plugin-content-docs/

# Backend (FastAPI RAG chatbot)
backend/
├── src/
│   ├── main.py                      # FastAPI app entry point
│   ├── models/
│   │   ├── conversation.py          # Conversation data models
│   │   ├── progress.py              # Progress tracking models
│   │   └── embeddings.py            # Vector embedding models
│   ├── services/
│   │   ├── chatbot_service.py       # RAG logic with OpenAI
│   │   ├── vector_service.py        # Qdrant vector search
│   │   ├── progress_service.py      # Progress tracking logic
│   │   └── content_indexer.py       # Index textbook content
│   ├── api/
│   │   ├── chatbot.py               # Chatbot endpoints
│   │   └── progress.py              # Progress endpoints
│   └── config.py                    # Configuration management
├── tests/
│   ├── test_chatbot.py
│   ├── test_progress.py
│   └── test_vector_search.py
├── requirements.txt
└── Dockerfile                       # Container for deployment

# Configuration
docusaurus.config.js                 # Docusaurus configuration
sidebars.js                          # Navigation structure
package.json                         # Frontend dependencies
tsconfig.json                        # TypeScript configuration

# CI/CD
.github/
└── workflows/
    ├── deploy.yml                   # Deploy to GitHub Pages
    ├── validate-content.yml         # Validate chapter structure
    └── test-backend.yml             # Backend API tests

# Environment
.env.example                         # Environment variables template
.gitignore
README.md
```

**Structure Decision**: Web application with separated frontend (Docusaurus static site) and backend (FastAPI API). Frontend deployed to GitHub Pages for free hosting and CDN distribution. Backend deployed to cloud platform (Vercel/Railway) for API endpoints. This separation allows independent scaling and deployment of content vs. dynamic features.

## Complexity Tracking

> **No violations detected** - Constitution check passed all gates. No complexity justification required.

## Phase 0: Research & Technology Decisions

*See [research.md](./research.md) for detailed findings*

### Key Decisions

1. **Static Site Generator: Docusaurus 3.x**
   - Rationale: Built for documentation, excellent performance, MDX support, i18n built-in
   - Alternatives: VuePress (less mature), GitBook (proprietary), custom Next.js (more complex)

2. **RAG Backend: FastAPI + OpenAI SDK**
   - Rationale: Fast async Python, OpenAI SDK for ChatKit, easy deployment
   - Alternatives: Node.js + LangChain (less Python ecosystem), Django (heavier)

3. **Vector Database: Qdrant Cloud**
   - Rationale: Managed service, generous free tier, excellent Python client
   - Alternatives: Pinecone (more expensive), Weaviate (self-hosted complexity)

4. **User Database: Neon Postgres**
   - Rationale: Serverless PostgreSQL, free tier, auto-scaling
   - Alternatives: Supabase (more features but heavier), MongoDB (schema flexibility not needed)

5. **Deployment: GitHub Pages + Vercel**
   - Rationale: Free hosting, automatic deployments, CDN distribution
   - Alternatives: Netlify (similar), AWS S3 (more complex), custom VPS (maintenance burden)

## Phase 1: Design Artifacts

### Data Model

*See [data-model.md](./data-model.md) for complete schemas*

**Core Entities:**
- Chapter (content metadata)
- Quiz (questions and answers)
- UserProgress (chapter completion, quiz scores)
- Conversation (chatbot history)
- Embedding (vector representations of content)

### API Contracts

*See [contracts/](./contracts/) for OpenAPI specifications*

**Chatbot API:**
- POST /api/chat - Send message to AI assistant
- GET /api/chat/history - Retrieve conversation history
- POST /api/chat/context - Set chapter context for queries

**Progress API:**
- GET /api/progress - Get user progress summary
- POST /api/progress/chapter - Mark chapter complete
- POST /api/progress/quiz - Submit quiz answers
- GET /api/progress/stats - Get learning statistics

### Developer Quickstart

*See [quickstart.md](./quickstart.md) for setup instructions*

**Prerequisites:**
- Node.js 18+, npm 9+
- Python 3.11+, pip
- Git

**Setup Steps:**
1. Clone repository
2. Install frontend dependencies: `npm install`
3. Install backend dependencies: `pip install -r backend/requirements.txt`
4. Configure environment variables
5. Start frontend: `npm start`
6. Start backend: `uvicorn backend.src.main:app --reload`

## Implementation Phases

### Phase 1: Content Foundation (MVP - P1)
**Goal**: Deliver 13 chapters with complete content structure

**Deliverables:**
- All 13 chapters with 12 mandatory sections each
- Docusaurus site with navigation and search
- Responsive design (375px+ width)
- Mermaid diagrams rendering
- Deploy to GitHub Pages

**Success Criteria:**
- All chapters accessible and properly formatted
- <2s page load time
- <1s search response time
- Mobile-responsive

### Phase 2: Interactive Learning (P2)
**Goal**: Add hands-on labs, exercises, and quizzes

**Deliverables:**
- Executable code examples with syntax highlighting
- Interactive quiz components
- Exercise sections with difficulty tiers
- Hardware tier documentation per chapter
- Code validation CI/CD

**Success Criteria:**
- 90%+ code examples execute successfully
- Quizzes functional with auto-grading
- Exercises categorized by difficulty

### Phase 3: AI Assistant (P3)
**Goal**: Embed RAG chatbot for contextual help

**Deliverables:**
- FastAPI backend with OpenAI integration
- Qdrant vector database with indexed content
- Chatbot widget in Docusaurus
- Conversation history persistence
- Context-aware text selection queries

**Success Criteria:**
- <3s response time (95% of queries)
- 90%+ accuracy (content-grounded responses)
- Conversation history preserved

### Phase 4: Progress Tracking (P4)
**Goal**: Track student progress and quiz performance

**Deliverables:**
- User authentication (optional accounts)
- Progress dashboard
- Quiz score tracking
- Chapter completion markers
- Learning recommendations

**Success Criteria:**
- Progress persisted across sessions
- Quiz scores recorded and displayed
- Recommendations based on performance

### Phase 5: Multilingual Support (P5)
**Goal**: Add Urdu (Roman script) translations

**Deliverables:**
- i18n configuration for Urdu
- Translated content for all chapters
- Language switcher UI
- Urdu support in AI assistant

**Success Criteria:**
- <2s language switching
- All content translated (except code)
- AI assistant responds in Urdu

## Risk Analysis

### Technical Risks

1. **OpenAI API Rate Limits**
   - Risk: High usage could exceed free tier quotas
   - Mitigation: Implement caching, rate limiting, fallback to search
   - Contingency: Budget for paid tier or switch to open-source LLM

2. **GitHub Pages Build Time**
   - Risk: Large site (156 pages) may exceed 10-minute build limit
   - Mitigation: Optimize images, lazy-load diagrams, incremental builds
   - Contingency: Switch to Vercel/Netlify with longer build times

3. **Code Example Validation**
   - Risk: ROS 2 examples require complex environment setup for CI/CD
   - Mitigation: Docker containers with ROS 2 pre-installed
   - Contingency: Manual validation with documented test procedures

4. **Vector Database Costs**
   - Risk: Qdrant free tier (1GB) may be insufficient for all content
   - Mitigation: Optimize embeddings, chunk content efficiently
   - Contingency: Self-host Qdrant or switch to Pinecone

### Content Risks

1. **Subject Matter Expertise**
   - Risk: Technical accuracy requires robotics/AI domain knowledge
   - Mitigation: SME review process before publication
   - Contingency: Community feedback mechanism for corrections

2. **Hardware Accessibility**
   - Risk: Students may lack access to recommended hardware
   - Mitigation: Emphasize cloud instances as minimum tier
   - Contingency: Provide detailed cloud setup guides with cost estimates

3. **Content Maintenance**
   - Risk: Robotics frameworks evolve (ROS 2 updates, new Isaac versions)
   - Mitigation: Version-specific documentation, update schedule
   - Contingency: Community contributions for updates

## Next Steps

1. ✅ Phase 0 complete: Research and technology decisions documented
2. ✅ Phase 1 complete: Data models, API contracts, quickstart guide created
3. ➡️ Ready for `/sp.tasks` to generate implementation task list
4. After tasks: Begin Phase 1 implementation (Content Foundation)

## Notes

- Constitution compliance verified - all 7 principles satisfied
- MVP focuses on P1 (content) + P2 (labs) for maximum educational value
- P3 (AI assistant) and P4 (progress) can be added incrementally
- P5 (Urdu) is optional enhancement, not blocking for launch
- Existing Docusaurus setup in repo provides head start on implementation
