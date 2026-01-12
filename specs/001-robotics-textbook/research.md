# Research & Technology Decisions

**Feature**: Physical AI & Humanoid Robotics Textbook Platform
**Date**: 2026-01-11
**Phase**: Phase 0 - Research

## Overview

This document captures research findings and technology decisions for building an AI-native educational platform. All decisions prioritize educational effectiveness, performance, accessibility, and maintainability while staying within free/low-cost tiers for initial deployment.

## Decision 1: Static Site Generator

### Context
Need a platform to deliver 13 chapters (156 pages) of educational content with fast load times, search functionality, responsive design, and multilingual support.

### Options Evaluated

**Docusaurus 3.x** ✅ SELECTED
- **Pros**: Built specifically for documentation, excellent performance (code splitting, prefetching), MDX support for interactive components, i18n plugin built-in, active Meta/Facebook maintenance, large plugin ecosystem
- **Cons**: React-based (learning curve for non-React developers), opinionated structure
- **Performance**: <2s page load achievable, <1s search with docusaurus-search-local
- **Cost**: Free (open source)

**VuePress 2.x**
- **Pros**: Vue-based, simpler than Docusaurus, good performance
- **Cons**: Smaller ecosystem, less mature than Docusaurus, fewer plugins for educational features
- **Rejected**: Less mature, smaller community support

**GitBook**
- **Pros**: Beautiful UI, excellent for documentation, built-in analytics
- **Cons**: Proprietary platform, limited free tier, less customization
- **Rejected**: Vendor lock-in, cost concerns at scale

**Custom Next.js**
- **Pros**: Maximum flexibility, excellent performance, large ecosystem
- **Cons**: Requires building everything from scratch (search, navigation, i18n), higher maintenance burden
- **Rejected**: Over-engineering for documentation needs

### Decision Rationale

Docusaurus 3.x selected because:
1. Purpose-built for documentation with all required features out-of-box
2. Proven performance at scale (used by React, Jest, Babel docs)
3. MDX support enables interactive quizzes and custom components
4. i18n plugin simplifies Urdu translation (P5 requirement)
5. Active development and strong community support
6. Existing setup in repo provides head start

## Decision 2: RAG Backend Framework

### Context
Need a backend API to serve RAG chatbot functionality with <3s response time, OpenAI integration, and easy deployment to cloud platforms.

### Options Evaluated

**FastAPI (Python 3.11+)** ✅ SELECTED
- **Pros**: Async/await for high performance, automatic OpenAPI docs, excellent Python ecosystem (langchain, qdrant-client), easy deployment to Vercel/Railway, type hints for safety
- **Cons**: Python (not JavaScript like frontend), requires separate deployment
- **Performance**: <3s response achievable with streaming, handles 1000+ concurrent requests
- **Cost**: Free tier available on Vercel/Railway

**Node.js + Express + LangChain**
- **Pros**: Same language as frontend (JavaScript/TypeScript), unified codebase
- **Cons**: LangChain.js less mature than Python version, smaller AI/ML ecosystem
- **Rejected**: Python ecosystem superior for AI/ML workloads

**Django + Django REST Framework**
- **Pros**: Batteries-included, excellent ORM, admin panel
- **Cons**: Heavier than FastAPI, slower startup, more complex for simple API
- **Rejected**: Over-engineering for API-only backend

**Serverless Functions (Vercel/Netlify)**
- **Pros**: Auto-scaling, pay-per-use, no server management
- **Cons**: Cold start latency, timeout limits (10-30s), stateless (harder for conversation context)
- **Rejected**: Cold starts conflict with <3s response requirement

### Decision Rationale

FastAPI selected because:
1. Async performance meets <3s response requirement
2. Python ecosystem best for AI/ML (OpenAI SDK, langchain, qdrant-client)
3. Automatic OpenAPI documentation aids development
4. Easy deployment to free tiers (Vercel, Railway, Render)
5. Type hints prevent common bugs
6. Streaming responses possible for better UX

## Decision 3: Vector Database

### Context
Need vector storage for semantic search over textbook content to ground RAG responses. Must support ~156 pages of content with fast retrieval (<1s).

### Options Evaluated

**Qdrant Cloud** ✅ SELECTED
- **Pros**: Managed service (no ops), generous free tier (1GB vectors), excellent Python client, fast retrieval (<100ms), supports filtering by metadata (chapter, section)
- **Cons**: Relatively new (less mature than Pinecone), free tier limits
- **Performance**: <100ms vector search, sufficient for <3s total response time
- **Cost**: Free tier (1GB), then $25/month

**Pinecone**
- **Pros**: Most mature vector DB, excellent performance, good docs
- **Cons**: Expensive ($70/month after free tier), free tier very limited (1 index, 100k vectors)
- **Rejected**: Cost prohibitive for educational project

**Weaviate**
- **Pros**: Open source, self-hostable, feature-rich
- **Cons**: Requires self-hosting (ops burden), more complex setup
- **Rejected**: Ops complexity conflicts with simplicity goal

**Chroma**
- **Pros**: Simple, embeddable, good for prototyping
- **Cons**: Not production-ready for cloud deployment, limited scaling
- **Rejected**: Not suitable for production deployment

### Decision Rationale

Qdrant Cloud selected because:
1. Managed service eliminates ops burden
2. Free tier sufficient for 156 pages of content
3. Fast retrieval (<100ms) supports <3s response requirement
4. Python client integrates easily with FastAPI
5. Metadata filtering enables chapter-specific context
6. Upgrade path available if free tier exceeded

## Decision 4: User Database

### Context
Need persistent storage for user progress (chapter completion, quiz scores) and conversation history. Must support free tier for initial deployment.

### Options Evaluated

**Neon Postgres** ✅ SELECTED
- **Pros**: Serverless PostgreSQL, generous free tier (1GB storage, 100 hours compute/month), auto-scaling, standard SQL, excellent Python support (psycopg2)
- **Cons**: Relatively new service, free tier compute limits
- **Performance**: Standard PostgreSQL performance, sufficient for 1000+ users
- **Cost**: Free tier, then $19/month

**Supabase**
- **Pros**: PostgreSQL + auth + realtime + storage, generous free tier, excellent DX
- **Cons**: More features than needed (over-engineering), heavier client libraries
- **Rejected**: Over-engineering for simple progress tracking

**MongoDB Atlas**
- **Pros**: Flexible schema, generous free tier (512MB), good Python support
- **Cons**: NoSQL not needed (structured data), less familiar for most developers
- **Rejected**: SQL better fit for structured progress/quiz data

**PlanetScale**
- **Pros**: MySQL-compatible, generous free tier, excellent performance
- **Cons**: MySQL (less feature-rich than PostgreSQL), branching model adds complexity
- **Rejected**: PostgreSQL preferred for JSON support and features

### Decision Rationale

Neon Postgres selected because:
1. Serverless model eliminates ops burden
2. Free tier sufficient for initial deployment (1000+ users)
3. Standard PostgreSQL (familiar, well-documented)
4. Auto-scaling handles traffic spikes
5. JSON support useful for flexible conversation history
6. Excellent Python ecosystem (psycopg2, SQLAlchemy)

## Decision 5: Deployment Strategy

### Context
Need to deploy frontend (static site) and backend (API) with 99%+ uptime, <2s page load, and minimal cost.

### Options Evaluated

**GitHub Pages (Frontend) + Vercel (Backend)** ✅ SELECTED
- **Pros**: Both free tiers, automatic deployments, CDN distribution, excellent DX, GitHub Pages 99.9% SLA
- **Cons**: Separate deployments for frontend/backend
- **Performance**: GitHub Pages CDN ensures <2s load, Vercel edge network for API
- **Cost**: Free for both

**Netlify (Frontend + Backend)**
- **Pros**: Unified platform, serverless functions, good free tier
- **Cons**: Serverless function cold starts, timeout limits
- **Rejected**: Cold starts conflict with <3s API response requirement

**Vercel (Frontend + Backend)**
- **Pros**: Unified platform, excellent Next.js support, edge functions
- **Cons**: Docusaurus not optimized for Vercel (built for static), serverless limitations
- **Rejected**: GitHub Pages better for pure static sites

**AWS S3 + CloudFront + Lambda**
- **Pros**: Maximum control, excellent performance, scalable
- **Cons**: Complex setup, cost management complexity, steeper learning curve
- **Rejected**: Over-engineering, complexity conflicts with simplicity goal

**Self-Hosted VPS**
- **Pros**: Full control, predictable costs
- **Cons**: Ops burden (updates, security, monitoring), no auto-scaling, uptime responsibility
- **Rejected**: Ops burden conflicts with focus on content

### Decision Rationale

GitHub Pages + Vercel selected because:
1. GitHub Pages perfect for static Docusaurus site (free, fast, reliable)
2. Vercel excellent for FastAPI deployment (free tier, auto-scaling)
3. Both provide automatic deployments from Git
4. CDN distribution ensures <2s page load globally
5. 99.9% SLA meets 99%+ uptime requirement
6. Zero ops burden (managed services)
7. Clear upgrade path if free tiers exceeded

## Decision 6: Diagram Rendering

### Context
Need to render ~30-50 technical diagrams for robotics concepts with support for light/dark modes.

### Options Evaluated

**Mermaid.js** ✅ SELECTED
- **Pros**: Text-based (version controllable), renders in browser, supports light/dark themes, wide diagram types (flowchart, sequence, class), Docusaurus plugin available
- **Cons**: Limited styling control, learning curve for syntax
- **Performance**: Client-side rendering, lazy-loadable
- **Cost**: Free (open source)

**Draw.io / Excalidraw**
- **Pros**: Visual editing, beautiful output, easy to use
- **Cons**: Binary files (not version-controllable), manual export to images, no programmatic generation
- **Rejected**: Binary files conflict with Git-based workflow

**PlantUML**
- **Pros**: Text-based, powerful for UML diagrams
- **Cons**: Requires server-side rendering or Java, less modern than Mermaid
- **Rejected**: Server-side rendering adds complexity

**Static Images (PNG/SVG)**
- **Pros**: Maximum control over appearance, no runtime rendering
- **Cons**: Not version-controllable (binary), manual updates, separate light/dark versions needed
- **Rejected**: Maintenance burden for updates

### Decision Rationale

Mermaid.js selected because:
1. Text-based diagrams version-controllable in Git
2. Docusaurus plugin provides seamless integration
3. Automatic light/dark mode support
4. Wide range of diagram types (flowchart, sequence, class, state)
5. No build-time or server-side rendering needed
6. Easy updates (edit text, not images)

## Decision 7: Search Implementation

### Context
Need fast search (<1s) across 156 pages of content without external dependencies.

### Options Evaluated

**docusaurus-search-local** ✅ SELECTED
- **Pros**: Client-side search (no backend needed), <1s response, indexes all content at build time, works offline, free
- **Cons**: Index size grows with content (but acceptable for 156 pages)
- **Performance**: <1s search, ~500KB index for 156 pages
- **Cost**: Free

**Algolia DocSearch**
- **Pros**: Excellent search quality, hosted service, free for open source
- **Cons**: Requires application approval, external dependency, not offline-capable
- **Rejected**: External dependency, approval process

**Typesense**
- **Pros**: Fast, typo-tolerant, self-hostable
- **Cons**: Requires backend service, ops burden
- **Rejected**: Adds backend complexity

**Custom Lunr.js**
- **Pros**: Flexible, client-side, well-established
- **Cons**: Requires custom integration, more work than docusaurus-search-local
- **Rejected**: docusaurus-search-local already uses Lunr.js

### Decision Rationale

docusaurus-search-local selected because:
1. Built specifically for Docusaurus (zero-config integration)
2. Client-side search (no backend, no external service)
3. <1s response time meets requirement
4. Works offline (progressive web app potential)
5. Free (no API costs)
6. Indexes all content automatically at build time

## Technology Stack Summary

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| Frontend Framework | Docusaurus 3.x | Purpose-built for docs, excellent performance, MDX support |
| Backend Framework | FastAPI (Python 3.11+) | Async performance, Python AI/ML ecosystem, easy deployment |
| Vector Database | Qdrant Cloud | Managed service, free tier, fast retrieval |
| User Database | Neon Postgres | Serverless PostgreSQL, free tier, auto-scaling |
| Frontend Deployment | GitHub Pages | Free, reliable (99.9% SLA), CDN distribution |
| Backend Deployment | Vercel | Free tier, auto-scaling, excellent DX |
| Diagrams | Mermaid.js | Text-based, version-controllable, light/dark support |
| Search | docusaurus-search-local | Client-side, <1s response, free |
| AI/LLM | OpenAI API (ChatKit) | Best-in-class quality, streaming support, reasonable pricing |

## Implementation Priorities

1. **Phase 1 (MVP)**: Docusaurus site with 13 chapters, search, responsive design
2. **Phase 2**: Interactive quizzes, code examples, exercises
3. **Phase 3**: RAG chatbot (FastAPI + Qdrant + OpenAI)
4. **Phase 4**: Progress tracking (Neon Postgres)
5. **Phase 5**: Urdu translation (i18n plugin)

## Risk Mitigation

1. **OpenAI API Costs**: Implement caching, rate limiting; fallback to search if quota exceeded
2. **Free Tier Limits**: Monitor usage; upgrade path documented for Qdrant ($25/mo) and Neon ($19/mo)
3. **Build Time**: Optimize images, lazy-load diagrams; switch to Vercel if GitHub Pages build time exceeded
4. **Code Validation**: Docker containers with ROS 2 for CI/CD; manual validation as fallback

## Next Steps

1. ✅ Research complete - all technology decisions documented
2. ➡️ Proceed to Phase 1: Create data models, API contracts, quickstart guide
3. After Phase 1: Ready for `/sp.tasks` to generate implementation tasks
