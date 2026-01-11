<!--
Sync Impact Report:
- Version change: [INITIAL] → 1.0.0
- New constitution created for Physical AI & Humanoid Robotics Textbook
- Principles established: 7 core principles
- Sections added: Core Principles, Content Standards, Development Workflow, Governance
- Templates status:
  ✅ spec-template.md - aligned with user story prioritization
  ✅ plan-template.md - aligned with technical context requirements
  ✅ tasks-template.md - aligned with phase-based implementation
  ⚠ README.md - may need minor updates to reference constitution
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy & Educational Rigor

All content MUST be technically accurate and pedagogically sound:
- Code examples MUST be production-ready, tested, and compatible with Ubuntu 22.04 LTS
- ROS 2 (Humble/Iron) code MUST follow official best practices and conventions
- Hardware specifications MUST clearly distinguish minimum, recommended, and premium tiers
- Theoretical concepts MUST be supported by diagrams (Mermaid), practical examples, and hands-on labs
- All claims about robotics systems, AI models, or simulation platforms MUST be verifiable

**Rationale**: Students rely on this textbook for career preparation. Inaccurate or outdated information damages learning outcomes and professional readiness.

### II. Structured Learning Progression

Every chapter MUST follow the mandatory template structure:
- Overview
- Learning Objectives (4-5 measurable points)
- Prerequisites
- Estimated Time (Reading/Lab/Exercises)
- Core Concepts (theory + diagrams)
- Hands-On Lab (step-by-step code)
- Code Examples (production-ready)
- Exercises (practice problems)
- Quiz (5-10 questions)
- FAQs
- Hardware Requirements
- Resources & Further Reading

**Rationale**: Consistent structure enables self-paced learning, ensures comprehensive coverage, and supports the RAG chatbot's ability to provide contextually relevant answers.

### III. Beginner-Friendly Progression (NON-NEGOTIABLE)

Content MUST progress from foundational to advanced concepts:
- Module 1 (Weeks 1-5): ROS 2 fundamentals before simulation
- Module 2 (Weeks 6-7): Physics simulation before photorealistic rendering
- Module 3 (Weeks 8-10): NVIDIA Isaac and advanced perception after ROS 2 mastery
- Module 4 (Weeks 11-13): Vision-Language-Action models only after perception foundations
- Each chapter MUST build on previous chapters' concepts
- Prerequisites MUST be explicitly stated and enforced

**Rationale**: Physical AI combines multiple complex domains (robotics, AI, simulation). Students without proper foundation will struggle with advanced topics, leading to frustration and dropout.

### IV. Hands-On Lab Quality

Every hands-on lab MUST be:
- **Executable**: Step-by-step instructions that work on specified hardware/cloud instances
- **Self-Contained**: All dependencies, setup steps, and expected outputs documented
- **Error-Handled**: Common errors anticipated with troubleshooting guidance
- **Validated**: Tested on minimum hardware tier before publication
- **Commented**: Code includes explanatory comments for learning purposes

**Rationale**: Practical skills differentiate this textbook from theory-only resources. Labs that fail to execute destroy student confidence and trust.

### V. Deployment & Accessibility

The platform MUST be:
- **Always Available**: Deployed to GitHub Pages with 99%+ uptime
- **Fast**: Page load <2 seconds, search results <1 second
- **Responsive**: Functional on devices 375px+ width
- **Multilingual**: English primary, Urdu (Roman) optional but properly implemented if included
- **Searchable**: Local search across all content with accurate results

**Rationale**: Educational content is worthless if inaccessible. Students in different regions, on different devices, must have equal access.

### VI. RAG Chatbot Integration

The embedded chatbot MUST:
- **Answer Accurately**: Responses grounded in textbook content (no hallucinations)
- **Support Context**: Handle user-selected text queries for targeted help
- **Persist State**: Use Neon Postgres + Qdrant for conversation history and embeddings
- **Respond Quickly**: <3 second response time for typical queries
- **Fail Gracefully**: Clear error messages when unable to answer, with fallback to search

**Rationale**: AI-native textbook means intelligent assistance. A broken or inaccurate chatbot is worse than no chatbot—it misleads students.

### VII. Code Standards & Compatibility

All code MUST adhere to:
- **Platform**: Ubuntu 22.04 LTS compatibility (primary target)
- **ROS Version**: ROS 2 Humble or Iron (explicitly documented per example)
- **Error Handling**: Production-ready with try-catch, validation, logging
- **Documentation**: Inline comments explaining robotics/AI concepts, not just code mechanics
- **Testing**: Critical paths tested (simulation launch, node communication, perception pipelines)
- **Dependencies**: Pinned versions in requirements.txt / package.xml with installation instructions

**Rationale**: Students will copy-paste code. Broken or insecure code damages their projects and learning. Professional-grade code teaches professional habits.

## Content Standards

### Chapter Completeness

- All 13 chapters MUST be complete before considering the textbook "done"
- Each chapter MUST include all 12 mandatory sections from the template
- Placeholder content (e.g., "TODO", "[Add content here]") is NOT acceptable in published chapters
- Diagrams MUST render correctly in both light and dark modes

### Hardware Guidance

Every chapter MUST specify:
- **Minimum**: Cloud instances (AWS, GCP, Azure) with specific instance types and estimated costs
- **Recommended**: RTX 4070 Ti + Jetson Orin Nano with performance expectations
- **Premium**: RTX 4090 + Jetson Orin NX + Unitree G1 with advanced capabilities unlocked

### Quiz & Exercise Quality

- Quizzes MUST test understanding, not memorization (application-level questions)
- Exercises MUST have three difficulty tiers: beginner, intermediate, advanced
- Solutions or hints MUST be available (in separate file or behind toggle)
- FAQs MUST address common student confusion points identified during testing

## Development Workflow

### Content Creation Process

1. **Draft**: Write chapter following template, mark unclear sections with `[NEEDS CLARIFICATION: reason]`
2. **Technical Review**: Validate all code examples execute successfully on minimum hardware
3. **Pedagogical Review**: Verify learning objectives are measurable and exercises align with objectives
4. **Integration Test**: Ensure chapter integrates with previous chapters (prerequisites met)
5. **RAG Preparation**: Verify FAQs and key concepts are structured for chatbot retrieval
6. **Publish**: Deploy to staging, validate rendering, then merge to main

### Code Example Standards

- Use consistent naming conventions across all chapters (e.g., `robot_controller`, not `robotController` in one chapter and `robot_ctrl` in another)
- Include expected output as comments or separate output blocks
- Provide troubleshooting section for common errors (e.g., "If you see 'No module named rclpy'...")
- Link to official documentation for complex APIs (ROS 2, Isaac Sim, etc.)

### Bonus Feature Implementation

Bonus features (Better-Auth, Personalization, Urdu Translation) MUST:
- NOT break core functionality if they fail
- Have feature flags for easy enable/disable
- Include their own documentation and testing
- Be clearly marked as "optional" in user-facing UI

## Governance

### Amendment Process

1. **Proposal**: Document proposed change with rationale in GitHub issue
2. **Impact Analysis**: Identify affected chapters, code examples, templates
3. **Approval**: Requires explicit sign-off (project owner or designated reviewer)
4. **Migration**: Update all affected content before merging constitution change
5. **Version Bump**: Follow semantic versioning (MAJOR.MINOR.PATCH)

### Versioning Policy

- **MAJOR**: Breaking changes to chapter structure, removal of mandatory sections, incompatible tech stack changes
- **MINOR**: New mandatory sections added, new principles added, expanded requirements
- **PATCH**: Clarifications, typo fixes, non-semantic improvements

### Compliance Review

- All PRs MUST verify compliance with this constitution
- Chapter PRs MUST include checklist confirming all 12 template sections present
- Code PRs MUST confirm Ubuntu 22.04 + ROS 2 compatibility
- Complexity or deviations MUST be justified in PR description

### Constitution Authority

- This constitution supersedes all other practices and guidelines
- When in conflict, constitution principles take precedence
- Runtime development guidance in `CLAUDE.md` MUST align with this constitution
- Templates in `.specify/templates/` MUST reflect these principles

**Version**: 1.0.0 | **Ratified**: 2026-01-11 | **Last Amended**: 2026-01-11
