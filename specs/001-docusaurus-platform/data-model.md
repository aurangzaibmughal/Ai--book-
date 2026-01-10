# Data Model: Docusaurus Platform Setup

**Feature**: 001-docusaurus-platform
**Date**: 2026-01-07
**Phase**: 1 - Design

## Overview

This document defines the content structure and configuration model for the Docusaurus-based AI textbook platform. Since Docusaurus is a static site generator, the "data model" consists of file-based content structures, configuration schemas, and metadata formats rather than traditional database entities.

## Core Entities

### 1. Chapter

**Description**: A single learning unit representing one chapter of the AI textbook. Each chapter must comply with the Chapter Quality Standards (Constitution Principle V).

**File Structure**:
```
docs/chapters/chapter-{number}-{slug}/
├── index.md                 # Chapter overview and introduction
├── learning-objectives.md   # Clear, measurable learning goals
├── concepts.md              # Core concepts with Mermaid diagrams
├── hands-on-lab.md          # Interactive coding exercise
├── exercises.md             # Practice problems (beginner/intermediate/advanced)
├── quiz.md                  # Knowledge check questions
└── faqs.md                  # Frequently asked questions (RAG-ready)
```

**Frontmatter Schema** (index.md):
```yaml
---
id: chapter-{number}-{slug}
title: "Chapter {number}: {Title}"
sidebar_label: "{Short Title}"
sidebar_position: {number}
description: "Brief chapter description for SEO"
tags: ["{topic1}", "{topic2}", "{difficulty}"]
---
```

**Required Sections** (per Constitution Principle V):
1. **Learning Objectives**: 3-5 measurable goals using Bloom's taxonomy verbs
2. **Concepts + Diagrams**: Explanations with at least 1 Mermaid diagram
3. **Tested Code**: Working code examples with verification steps
4. **Hands-On Lab**: Step-by-step interactive exercise with solution
5. **Pitfalls**: Common mistakes and how to avoid them
6. **Exercises**: 3 levels (beginner, intermediate, advanced)
7. **Quiz**: 5-10 multiple choice or short answer questions
8. **FAQs**: 5-10 Q&A pairs optimized for RAG retrieval

**Validation Rules**:
- All 7 section files must exist
- Each section must have minimum content length (>100 words)
- At least 1 Mermaid diagram in concepts.md
- Code blocks must have language tags for syntax highlighting
- Quiz questions must have correct answers marked

**Relationships**:
- Belongs to a Category (via sidebar configuration)
- Has translations in Language Locales (optional)
- Referenced by Navigation Structure

### 2. Language Locale

**Description**: Represents a supported language with associated translations for UI elements and content.

**File Structure**:
```
i18n/{locale-code}/
├── docusaurus-plugin-content-docs/
│   └── current/
│       └── chapters/
│           └── chapter-{number}-{slug}/
│               ├── index.md
│               ├── learning-objectives.md
│               └── ... (all chapter sections)
├── docusaurus-theme-classic/
│   ├── navbar.json          # Navigation bar translations
│   ├── footer.json          # Footer translations
│   └── docs.json            # Docs plugin translations
└── code.json                # Custom component translations
```

**Supported Locales**:
- `en` (English) - Default locale, LTR direction
- `ur` (Urdu) - Additional locale, RTL direction

**Configuration Schema** (docusaurus.config.js):
```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en-US',
    },
    ur: {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur-PK',
    },
  },
}
```

**Attributes**:
- `code`: ISO 639-1 language code (e.g., 'en', 'ur')
- `label`: Display name in native language
- `direction`: Text direction ('ltr' or 'rtl')
- `htmlLang`: HTML lang attribute value
- `path`: URL path prefix (e.g., '/ur/')

**Validation Rules**:
- Locale code must be valid ISO 639-1
- Direction must be 'ltr' or 'rtl'
- Translation files must match source structure
- Missing translations fall back to default locale

**Relationships**:
- Contains translations for Chapters
- Referenced by Navigation Structure
- Used by Language Switcher component

### 3. Navigation Structure

**Description**: Hierarchical organization of chapters in the sidebar navigation.

**File**: `sidebars.js`

**Schema**:
```javascript
module.exports = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Getting Started',
      collapsible: true,
      collapsed: false,
      items: [
        'chapters/chapter-01-introduction/index',
        'chapters/chapter-02-setup/index',
      ],
    },
    {
      type: 'category',
      label: 'Core Concepts',
      collapsible: true,
      collapsed: true,
      items: [
        'chapters/chapter-03-fundamentals/index',
        'chapters/chapter-04-advanced/index',
      ],
    },
  ],
};
```

**Node Types**:
- `doc`: Single document link
- `category`: Collapsible group of items
- `link`: External link
- `autogenerated`: Auto-generate from directory structure

**Attributes**:
- `type`: Node type (doc, category, link, autogenerated)
- `id`: Document ID (relative path without extension)
- `label`: Display text in sidebar
- `collapsible`: Whether category can be collapsed (boolean)
- `collapsed`: Initial collapsed state (boolean)
- `items`: Child nodes (array)

**Validation Rules**:
- All referenced document IDs must exist
- No circular references
- Maximum nesting depth: 3 levels
- Category must have at least 1 item

**Relationships**:
- References Chapters by ID
- Supports multiple locales (separate sidebars per locale)
- Rendered by Sidebar component

### 4. Site Configuration

**Description**: Global configuration for the Docusaurus site.

**File**: `docusaurus.config.js`

**Key Sections**:

**Site Metadata**:
```javascript
{
  title: 'AI Textbook Platform',
  tagline: 'Learn AI with Interactive Tutorials',
  url: 'https://{username}.github.io',
  baseUrl: '/{repo-name}/',
  organizationName: '{github-username}',
  projectName: '{repo-name}',
  favicon: 'img/favicon.ico',
}
```

**Theme Configuration**:
```javascript
themeConfig: {
  navbar: {
    title: 'AI Textbook',
    logo: { alt: 'Logo', src: 'img/logo.svg' },
    items: [
      { type: 'doc', docId: 'intro', label: 'Chapters' },
      { type: 'localeDropdown', position: 'right' },
      { type: 'search', position: 'right' },
    ],
  },
  footer: {
    style: 'dark',
    links: [...],
    copyright: '...',
  },
  prism: {
    theme: lightCodeTheme,
    darkTheme: darkCodeTheme,
    additionalLanguages: ['python', 'javascript', 'typescript'],
  },
  colorMode: {
    defaultMode: 'light',
    disableSwitch: false,
    respectPrefersColorScheme: true,
  },
}
```

**Plugins**:
```javascript
plugins: [
  '@docusaurus/plugin-content-docs',
  '@docusaurus/plugin-content-pages',
  '@docusaurus/plugin-sitemap',
]
```

**Presets**:
```javascript
presets: [
  [
    'classic',
    {
      docs: {
        sidebarPath: require.resolve('./sidebars.js'),
        editUrl: 'https://github.com/{org}/{repo}/edit/main/',
      },
      theme: {
        customCss: require.resolve('./src/css/custom.css'),
      },
    },
  ],
]
```

**Validation Rules**:
- URL must be valid HTTPS URL
- baseUrl must start and end with '/'
- All plugin names must be valid npm packages
- Theme configuration must match theme schema

### 5. Custom Components

**Description**: React components extending Docusaurus functionality.

**Component: ChapterTemplate**

**Purpose**: Standardized layout for chapter pages with navigation between sections

**Props**:
```typescript
interface ChapterTemplateProps {
  chapterId: string;
  sections: Array<{
    id: string;
    title: string;
    path: string;
  }>;
  currentSection: string;
}
```

**Component: LanguageSwitcher**

**Purpose**: Custom language selector with locale persistence

**Props**:
```typescript
interface LanguageSwitcherProps {
  currentLocale: string;
  availableLocales: Array<{
    code: string;
    label: string;
    direction: 'ltr' | 'rtl';
  }>;
  onLocaleChange: (locale: string) => void;
}
```

**Component: MermaidDiagram**

**Purpose**: Wrapper for Mermaid diagrams with error handling

**Props**:
```typescript
interface MermaidDiagramProps {
  chart: string;
  caption?: string;
  alt: string;
}
```

## Content Workflow

### 1. Chapter Creation

```
1. Create chapter directory: docs/chapters/chapter-{N}-{slug}/
2. Copy chapter template files (7 required sections)
3. Fill frontmatter in index.md
4. Write content for each section
5. Add Mermaid diagrams to concepts.md
6. Validate chapter completeness (linting)
7. Update sidebars.js with new chapter
8. Commit and push (triggers deployment)
```

### 2. Translation Workflow

```
1. Complete English version of chapter
2. Create translation directory: i18n/ur/docusaurus-plugin-content-docs/current/chapters/chapter-{N}-{slug}/
3. Copy English files to translation directory
4. Translate content (preserve Markdown structure)
5. Update UI translations in i18n/ur/docusaurus-theme-classic/
6. Test RTL rendering
7. Commit and push
```

### 3. Navigation Update

```
1. Edit sidebars.js
2. Add new chapter to appropriate category
3. Set sidebar_position in chapter frontmatter
4. Test navigation locally
5. Commit and push
```

## File System Constraints

**GitHub Pages Limits**:
- Maximum repository size: 1GB
- Maximum file size: 100MB
- Build timeout: 10 minutes

**Docusaurus Limits**:
- Recommended max pages: 1000 (performance consideration)
- Sidebar items: No hard limit, but >100 items may impact UX
- Asset size: Keep images <500KB, optimize for web

**Best Practices**:
- Use image optimization (WebP format, compression)
- Lazy load images and diagrams
- Split large chapters into multiple pages
- Monitor build size in CI

## Validation and Linting

**Chapter Validation Rules**:
1. All 7 required section files exist
2. Frontmatter schema is valid
3. At least 1 Mermaid diagram in concepts.md
4. Code blocks have language tags
5. Internal links are valid
6. Images have alt text

**Implementation**:
- Custom ESLint plugin for Markdown validation
- Pre-commit hooks for validation
- CI checks before deployment

## Future Extensions

**For RAG Chatbot Integration**:
- Extract FAQ sections into JSON format
- Add metadata for vector embedding (chapter, topic, difficulty)
- Generate content chunks with context

**For Authentication Integration**:
- Add user progress tracking (completed chapters, quiz scores)
- Personalized content recommendations
- Bookmark and note-taking features

## Conclusion

This data model provides a structured approach to content organization while maintaining flexibility for future enhancements. The file-based structure aligns with Docusaurus conventions and supports the constitution's chapter quality standards.
