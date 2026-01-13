# Physical AI & Humanoid Robotics text-book

[![Deploy to GitHub Pages](https://github.com/your-username/ai-book/actions/workflows/deploy.yml/badge.svg)](https://github.com/your-username/ai-book/actions/workflows/deploy.yml)

An interactive Physical AI and Humanoid Robotics learning platform built with [Docusaurus](https://docusaurus.io/), featuring comprehensive tutorials on robot manipulation, locomotion, perception, and control with multilingual support.

## 🌟 Features

- **📚 Comprehensive Content**: Each chapter includes learning objectives, concepts, diagrams, hands-on labs, exercises, quizzes, and FAQs
- **🤖 Robotics Focus**: Learn Physical AI, humanoid robotics, manipulation, locomotion, and embodied intelligence
- **🎨 Interactive Learning**: Mermaid diagrams, code examples, and practical robotics exercises
- **🌍 Multilingual Support**: English and Urdu with RTL text support
- **🔍 Fast Search**: Local search across all content
- **🎨 Dark Mode**: Toggle between light and dark themes
- **📱 Responsive Design**: Works on desktop, tablet, and mobile devices
- **🚀 Fast Performance**: Optimized for speed with code splitting and prefetching

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18.0 or higher
- **npm** 9.0 or higher (or **yarn** 1.22+)
- **Git** 2.0 or higher

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/your-username/ai-book.git
   cd ai-book
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

   If you encounter issues, try:
   ```bash
   npm install --legacy-peer-deps
   ```

   Or use yarn:
   ```bash
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   ```

   The site will open at [http://localhost:3000](http://localhost:3000)

## 📖 Usage

### Development

```bash
# Start development server with hot reload
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Clear cache
npm run clear
```

### Creating New Chapters

Each chapter must include these 7 required sections:

1. **index.md** - Chapter overview
2. **learning-objectives.md** - 3-5 measurable learning goals
3. **concepts.md** - Core concepts with Mermaid diagrams
4. **hands-on-lab.md** - Interactive coding exercise
5. **exercises.md** - Practice problems (beginner/intermediate/advanced)
6. **quiz.md** - Knowledge check questions
7. **faqs.md** - Frequently asked questions (RAG-ready)

**Example structure:**
```
docs/chapters/chapter-02-machine-learning/
├── index.md
├── learning-objectives.md
├── concepts.md
├── hands-on-lab.md
├── exercises.md
├── quiz.md
└── faqs.md
```

**Update `sidebars.js`** to include the new chapter:
```javascript
{
  type: 'category',
  label: 'Machine Learning',
  items: [
    'chapters/chapter-02-machine-learning/index',
  ],
}
```

## 🌐 Deployment

### GitHub Pages (Automatic)

The site automatically deploys to GitHub Pages when you push to the `main` branch.

**Setup Steps:**

1. **Update `docusaurus.config.js`** with your repository details:
   ```javascript
   url: 'https://your-username.github.io',
   baseUrl: '/ai-book/',
   organizationName: 'your-username',
   projectName: 'ai-book',
   ```

2. **Enable GitHub Pages** in repository settings:
   - Go to Settings → Pages
   - Source: Deploy from a branch
   - Branch: `gh-pages` / `root`
   - Save

3. **Push to main branch**:
   ```bash
   git add .
   git commit -m "Initial deployment"
   git push origin main
   ```

4. **Check deployment status**:
   - Go to Actions tab in GitHub
   - View the "Deploy to GitHub Pages" workflow
   - Once complete, visit `https://your-username.github.io/ai-book/`

### Manual Deployment

```bash
# Set your GitHub username
GIT_USER=your-username npm run deploy
```

## 🌍 Internationalization

### Adding Translations

1. **Create translation directory**:
   ```bash
   mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
   ```

2. **Copy content to translate**:
   ```bash
   cp -r docs/* i18n/ur/docusaurus-plugin-content-docs/current/
   ```

3. **Translate the content** while preserving Markdown structure

4. **Add UI translations**:
   ```bash
   npm run write-translations -- --locale ur
   ```

5. **Test translations**:
   ```bash
   npm run start -- --locale ur
   ```

## 📁 Project Structure

```
ai-book/
├── .github/
│   └── workflows/
│       └── deploy.yml          # GitHub Actions deployment
├── docs/                       # Documentation content
│   ├── intro.md               # Homepage
│   └── chapters/              # Chapter content
│       └── chapter-01-introduction/
├── i18n/                      # Translations
│   └── ur/                    # Urdu translations
├── src/
│   ├── components/            # Custom React components
│   ├── css/                   # Custom styles
│   └── pages/                 # Custom pages
├── static/                    # Static assets
│   ├── img/                   # Images
│   └── fonts/                 # Fonts
├── docusaurus.config.js       # Site configuration
├── sidebars.js                # Navigation structure
├── package.json               # Dependencies
└── tsconfig.json              # TypeScript config
```

## 🛠️ Configuration

### Site Metadata

Edit `docusaurus.config.js`:

```javascript
{
  title: 'Your Site Title',
  tagline: 'Your tagline',
  url: 'https://your-domain.com',
  baseUrl: '/',
  // ... other settings
}
```

### Theme Customization

Edit `src/css/custom.css`:

```css
:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  /* ... other colors */
}
```

### Navigation

Edit `sidebars.js` to customize the sidebar structure.

## 🧪 Testing

### Manual Testing Checklist

- [ ] All pages load without errors
- [ ] Mermaid diagrams render correctly
- [ ] Search functionality works
- [ ] Navigation highlights current page
- [ ] Responsive design works on mobile (375px+)
- [ ] Dark mode toggle works
- [ ] Language switcher works
- [ ] All internal links are valid

### Performance Testing

```bash
# Install Lighthouse CI
npm install -g @lhci/cli

# Run Lighthouse audit
lhci autorun
```

**Target Scores:**
- Performance: 90+
- Accessibility: 90+
- Best Practices: 90+
- SEO: 90+

## 📊 Success Criteria

- ✅ Setup time: <10 minutes
- ✅ Navigation: <3 clicks to any chapter
- ✅ Page load: <2 seconds
- ✅ Deployment: <5 minutes
- ✅ Search results: <1 second
- ✅ Language switch: Instant
- ✅ Mobile support: 375px+ width
- ✅ Diagram rendering: 100%
- ✅ Lighthouse score: 90+
- ✅ Broken links: Zero

## 📐 Development Methodology

This project follows **Specification-Driven Development (SpecKitPlus)**, an AI-driven documentation engineering methodology that emphasizes:

- **Spec is Law**: All content follows defined specifications and templates
- **Structure First, Content Second**: Validate structure before generating content
- **Deterministic Output**: Same input specification produces same structure
- **Code Quality Guarantee**: All code examples are complete, runnable, and tested
- **Module-Based Thinking**: Work within defined module boundaries with explicit dependencies

### Constitution

The project constitution defines core principles, workflow enforcement, and governance policies. See [`.specify/memory/constitution.md`](.specify/memory/constitution.md) for complete details.

**Current Version**: 2.0.0 (Last Amended: 2026-01-13)

### Development Workflow

All development follows the SpecKitPlus workflow:
1. Load and validate specification
2. Confirm template structure compliance
3. Generate content following validated structure
4. Test all code examples for executability
5. Cross-reference with previous modules
6. Execute validation checklist before completion

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Content Guidelines

- **Follow the constitution**: All content MUST comply with [`.specify/memory/constitution.md`](.specify/memory/constitution.md)
- Follow the chapter template structure exactly
- Include all 7 required sections (no placeholders in published content)
- Add Mermaid diagrams for visual concepts
- Provide working code examples (Python-first, Ubuntu 22.04 LTS compatible)
- Write clear, concise explanations (bilingual-ready English)
- Test all code before committing
- Validate structure before content generation

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Diagrams powered by [Mermaid](https://mermaid.js.org/)
- Search by [@easyops-cn/docusaurus-search-local](https://github.com/easyops-cn/docusaurus-search-local)
- Deployed with [GitHub Pages](https://pages.github.com/)

## 📞 Support

- **Documentation**: [Docusaurus Docs](https://docusaurus.io/docs)
- **Issues**: [GitHub Issues](https://github.com/your-username/ai-book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/ai-book/discussions)

## 🗺️ Roadmap

- [x] Phase 1: Basic documentation site structure
- [x] Phase 2: Automated GitHub Pages deployment
- [ ] Phase 3: Urdu language support
- [ ] Phase 4: RAG chatbot integration
- [ ] Phase 5: Authentication and personalization
- [ ] Phase 6: Additional chapters and content

---

**Built with ❤️ for AI learners worldwide**
