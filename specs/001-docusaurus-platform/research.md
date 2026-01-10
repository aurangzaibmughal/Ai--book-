# Research: Docusaurus Platform Setup

**Feature**: 001-docusaurus-platform
**Date**: 2026-01-07
**Phase**: 0 - Technology Research and Decision Making

## Overview

This document captures the research and decision-making process for establishing the AI textbook platform using Docusaurus. The goal is to create a production-ready documentation site with GitHub Pages deployment, multilingual support (English/Urdu), and extensibility for future RAG chatbot integration.

## Technology Decisions

### 1. Static Site Generator: Docusaurus 3.x

**Decision**: Use Docusaurus 3.x as the primary static site generator

**Rationale**:
- **Built for Documentation**: Specifically designed for technical documentation with excellent Markdown support
- **React-Based**: Allows custom components for chapter templates and interactive elements
- **Built-in Features**: Native support for versioning, search, i18n, dark mode, and responsive design
- **Mermaid Support**: Official plugin (@docusaurus/theme-mermaid) for diagram rendering
- **Active Maintenance**: Meta (Facebook) backed project with strong community support
- **Performance**: Optimized for fast page loads with code splitting and prefetching
- **GitHub Pages Integration**: First-class support with official deployment guides

**Alternatives Considered**:
- **VuePress**: Good documentation tool but smaller ecosystem and less active development
- **MkDocs**: Python-based, excellent for Python projects but less flexible for custom React components
- **GitBook**: Commercial product with limitations on free tier, less customizable
- **Nextra**: Newer Next.js-based option but less mature, fewer plugins for our needs

**Best Practices**:
- Use TypeScript for type safety in custom components
- Follow Docusaurus plugin architecture for extensibility
- Leverage MDX for interactive content (code playgrounds, quizzes)
- Use Docusaurus preset-classic for standard documentation features

### 2. Internationalization: Docusaurus i18n Plugin

**Decision**: Use built-in Docusaurus i18n plugin for English/Urdu support

**Rationale**:
- **Native Integration**: Built into Docusaurus core, no external dependencies
- **File-Based Translations**: Organized structure in `i18n/` directory
- **RTL Support**: Configurable text direction per locale
- **Locale Persistence**: Automatic language preference storage in localStorage
- **Translation Workflow**: Clear separation between default locale and translations

**Implementation Approach**:
- Default locale: English (`en`)
- Additional locale: Urdu (`ur`) with RTL direction
- Translation files organized by plugin (docs, theme, pages)
- Custom CSS for RTL-specific styling (text alignment, navigation)
- Urdu web fonts loaded via Google Fonts or local hosting

**Best Practices**:
- Use locale-specific configuration in `docusaurus.config.js`
- Provide fallback to English for missing translations
- Test RTL layout thoroughly (sidebar, navigation, code blocks)
- Document translation workflow for content authors

### 3. Diagram Rendering: @docusaurus/theme-mermaid

**Decision**: Use official Mermaid plugin for diagram rendering

**Rationale**:
- **Official Plugin**: Maintained by Docusaurus team, guaranteed compatibility
- **Markdown Integration**: Diagrams defined in code blocks with `mermaid` language tag
- **Client-Side Rendering**: No build-time dependencies, renders in browser
- **Wide Diagram Support**: Flowcharts, sequence diagrams, class diagrams, state diagrams, etc.
- **Customizable Themes**: Supports light/dark mode themes

**Configuration**:
```javascript
themes: ['@docusaurus/theme-mermaid'],
markdown: {
  mermaid: true,
},
```

**Best Practices**:
- Validate Mermaid syntax before committing (use Mermaid Live Editor)
- Provide alt text for accessibility
- Keep diagrams simple and focused (split complex diagrams)
- Use consistent styling across all diagrams

### 4. Deployment: GitHub Actions + GitHub Pages

**Decision**: Use GitHub Actions for CI/CD with GitHub Pages hosting

**Rationale**:
- **Free Hosting**: GitHub Pages is free for public repositories
- **Automated Deployment**: GitHub Actions triggers on push to main branch
- **Official Support**: Docusaurus provides official deployment workflow
- **Build Validation**: CI runs tests and build checks before deployment
- **Fast Deployment**: Typically completes within 2-5 minutes

**Workflow Configuration**:
- Trigger: Push to `main` branch
- Build: `npm ci && npm run build`
- Deploy: Upload build artifacts to `gh-pages` branch
- Environment: Node.js 18 LTS

**Best Practices**:
- Cache npm dependencies for faster builds
- Run Lighthouse CI in workflow for performance validation
- Set up branch protection rules (require passing checks)
- Configure custom domain if needed (CNAME file)

### 5. Testing Strategy

**Decision**: Multi-layer testing approach

**Testing Layers**:

1. **Unit Tests (Jest)**:
   - Custom React components (ChapterTemplate, LanguageSwitcher)
   - Utility functions (navigation helpers, i18n utilities)
   - Configuration validation

2. **E2E Tests (Playwright)**:
   - Navigation flows (sidebar, search, language switching)
   - Content rendering (Markdown, Mermaid diagrams, code blocks)
   - Responsive design (mobile, tablet, desktop)
   - RTL layout validation for Urdu

3. **Performance Tests (Lighthouse CI)**:
   - Performance score: 90+
   - Accessibility score: 90+
   - Best practices score: 90+
   - SEO score: 90+

**Best Practices**:
- Run tests in CI before deployment
- Test on multiple browsers (Chrome, Firefox, Safari)
- Validate mobile experience (375px minimum width)
- Monitor bundle size (keep under 500KB for main bundle)

### 6. Content Structure: Chapter Template Pattern

**Decision**: Standardized chapter structure with MDX components

**Chapter Structure**:
```
docs/chapters/chapter-XX/
├── index.md                 # Chapter overview
├── learning-objectives.md   # Clear, measurable goals
├── concepts.md              # Explanations + Mermaid diagrams
├── hands-on-lab.md          # Interactive exercise
├── exercises.md             # Beginner/intermediate/advanced
├── quiz.md                  # Knowledge check questions
└── faqs.md                  # RAG-ready Q&A pairs
```

**Rationale**:
- **Constitution Compliance**: Matches Chapter Quality Standards (Principle V)
- **Modular Content**: Each section is independently editable
- **Reusable Components**: MDX allows custom React components for quizzes, labs
- **RAG-Ready**: FAQ structure optimized for vector embedding and retrieval

**Best Practices**:
- Use frontmatter for metadata (title, description, tags)
- Implement custom MDX components for interactive elements
- Validate chapter completeness with linting rules
- Generate chapter templates with CLI tool

### 7. Search: Docusaurus Search Plugin

**Decision**: Use Algolia DocSearch (free for open source) or local search plugin

**Primary Option - Algolia DocSearch**:
- **Pros**: Fast, cloud-hosted, excellent UX, free for open source
- **Cons**: Requires application approval, external dependency
- **Performance**: <1s search results (meets SC-005)

**Fallback Option - @easyops-cn/docusaurus-search-local**:
- **Pros**: No external dependencies, works offline, privacy-friendly
- **Cons**: Larger bundle size, client-side indexing
- **Performance**: Acceptable for <100 pages

**Decision**: Start with local search, migrate to Algolia if approved

**Best Practices**:
- Index all content including FAQs for RAG preparation
- Configure search to include Urdu content
- Optimize search index size (exclude unnecessary content)
- Test search performance with realistic content volume

## Integration Points for Future Features

### RAG Chatbot Integration

**Preparation**:
- FAQ sections structured as Q&A pairs (easy to extract for vector DB)
- Consistent heading hierarchy for content chunking
- Metadata in frontmatter for filtering (chapter, difficulty, topic)
- API endpoint placeholder for chatbot widget

**Integration Approach**:
- Embed FastAPI chatbot widget as custom React component
- Use Docusaurus plugin system for chatbot configuration
- Maintain separation: Docusaurus for content, FastAPI for RAG backend
- Share content via build-time extraction or API

### Authentication Integration

**Preparation**:
- Design for optional authentication (site works without login)
- Plan for user-specific features (personalization, progress tracking)
- Consider SSO integration points (OAuth providers)

**Integration Approach**:
- Better-Auth SDK integrated as Docusaurus plugin
- Custom React components for login/signup UI
- Protected routes for personalized content
- User preferences stored in backend (Neon Postgres)

## Risk Mitigation

### Risk 1: GitHub Pages Build Failures

**Mitigation**:
- Comprehensive CI testing before deployment
- Build size monitoring (GitHub Pages has 1GB limit)
- Fallback deployment option (Netlify, Vercel)
- Clear error messages in GitHub Actions logs

### Risk 2: RTL Layout Issues

**Mitigation**:
- Dedicated RTL stylesheet (rtl.css)
- Manual testing with Urdu content
- Browser testing (Chrome, Firefox, Safari)
- Component-level RTL testing in Playwright

### Risk 3: Performance Degradation with Content Growth

**Mitigation**:
- Code splitting by chapter
- Lazy loading for images and diagrams
- Lighthouse CI monitoring in every deployment
- Bundle size budgets enforced in CI

### Risk 4: Mermaid Diagram Rendering Failures

**Mitigation**:
- Syntax validation in pre-commit hooks
- Fallback to static images if rendering fails
- Error boundaries around Mermaid components
- Clear error messages for authors

## Implementation Phases

### Phase 1: Core Platform (User Story 1 - P1)
- Docusaurus installation and configuration
- Basic chapter structure and navigation
- Mermaid diagram support
- Local development environment
- **Deliverable**: Working local site with sample chapter

### Phase 2: Deployment Pipeline (User Story 2 - P2)
- GitHub Actions workflow
- GitHub Pages configuration
- Build optimization
- Performance validation
- **Deliverable**: Publicly accessible site with automated deployment

### Phase 3: Internationalization (User Story 3 - P3)
- i18n plugin configuration
- Urdu locale setup
- RTL styling
- Language switcher component
- **Deliverable**: Bilingual site with language persistence

## Success Metrics Validation

| Success Criterion | Implementation Approach | Validation Method |
|-------------------|------------------------|-------------------|
| SC-001: Setup <10 min | Automated setup script, clear documentation | Time setup process with fresh environment |
| SC-002: Navigate <3 clicks | Hierarchical sidebar, homepage links | Manual navigation testing |
| SC-003: Load <2s | Code splitting, image optimization | Lighthouse CI, WebPageTest |
| SC-004: Deploy <5 min | Optimized GitHub Actions workflow | Monitor deployment times |
| SC-005: Search <1s | Efficient search indexing | Performance profiling |
| SC-006: Instant language switch | Client-side locale switching | Manual testing, E2E tests |
| SC-007: Mobile 375px+ | Responsive design, mobile-first CSS | Browser DevTools, real devices |
| SC-008: 100% Mermaid render | Syntax validation, error handling | Automated diagram tests |
| SC-009: Lighthouse 90+ | Performance optimization | Lighthouse CI in every build |
| SC-010: Zero broken links | Link validation in CI | Automated link checker |

## Conclusion

The technology stack is well-suited for the AI textbook platform requirements. Docusaurus provides a solid foundation with excellent documentation features, while maintaining flexibility for future RAG chatbot and authentication integration. All decisions align with the constitution principles and support the hackathon scoring criteria.

**Next Steps**: Proceed to Phase 1 design (data-model.md, quickstart.md) and begin implementation with `/sp.tasks`.
