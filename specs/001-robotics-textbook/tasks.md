# Tasks: Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `/specs/001-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: Repository root (Docusaurus site)
- **Backend**: `backend/src/` for FastAPI application
- **Content**: `docs/chapters/` for Markdown content
- **Components**: `src/components/` for React components

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Verify Docusaurus installation and dependencies in package.json
- [ ] T002 [P] Configure docusaurus.config.js with site metadata, theme, and plugins
- [ ] T003 [P] Configure sidebars.js with navigation structure for 13 chapters
- [ ] T004 [P] Setup custom CSS in src/css/custom.css for theme customization
- [ ] T005 Create backend directory structure: backend/src/{models,services,api,scripts}
- [ ] T006 [P] Initialize backend/requirements.txt with FastAPI, OpenAI SDK, langchain, qdrant-client, psycopg2
- [ ] T007 [P] Create backend/.env.example with required environment variables
- [ ] T008 [P] Setup backend/src/config.py for configuration management
- [ ] T009 [P] Create .github/workflows/deploy.yml for GitHub Pages deployment
- [ ] T010 [P] Create .github/workflows/validate-content.yml for content structure validation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T011 Create chapter template structure in docs/chapters/ with 12 mandatory sections
- [ ] T012 [P] Implement content validation script to verify 12-section structure per chapter
- [ ] T013 [P] Configure docusaurus-search-local plugin in docusaurus.config.js
- [ ] T014 [P] Setup Mermaid.js plugin for diagram rendering in docusaurus.config.js
- [ ] T015 Create database schema SQL in backend/src/scripts/init_db.py (users, user_progress, quiz_attempts, conversations, messages tables)
- [ ] T016 [P] Implement backend/src/models/user.py with User model
- [ ] T017 [P] Implement backend/src/models/conversation.py with Conversation and Message models
- [ ] T018 [P] Setup FastAPI app structure in backend/src/main.py with CORS, error handling, and routing
- [ ] T019 [P] Create backend/src/services/database.py for Neon Postgres connection management

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Structured Learning Content (Priority: P1) 🎯 MVP

**Goal**: Deliver 13 chapters with complete content structure, navigation, search, and responsive design

**Independent Test**: Navigate through all 13 chapters, verify each contains 12 sections, search for content, test on mobile (375px+ width)

### Implementation for User Story 1

- [ ] T020 [P] [US1] Create docs/chapters/chapter-01-introduction/index.md with Overview section
- [ ] T021 [P] [US1] Create docs/chapters/chapter-01-introduction/learning-objectives.md
- [ ] T022 [P] [US1] Create docs/chapters/chapter-01-introduction/prerequisites.md
- [ ] T023 [P] [US1] Create docs/chapters/chapter-01-introduction/estimated-time.md
- [ ] T024 [P] [US1] Create docs/chapters/chapter-01-introduction/concepts.md with Mermaid diagrams
- [ ] T025 [P] [US1] Create docs/chapters/chapter-01-introduction/hands-on-lab.md
- [ ] T026 [P] [US1] Create docs/chapters/chapter-01-introduction/code-examples.md
- [ ] T027 [P] [US1] Create docs/chapters/chapter-01-introduction/exercises.md
- [ ] T028 [P] [US1] Create docs/chapters/chapter-01-introduction/quiz.md
- [ ] T029 [P] [US1] Create docs/chapters/chapter-01-introduction/faqs.md
- [ ] T030 [P] [US1] Create docs/chapters/chapter-01-introduction/hardware-requirements.md
- [ ] T031 [P] [US1] Create docs/chapters/chapter-01-introduction/resources.md
- [ ] T032 [P] [US1] Create docs/chapters/chapter-02-ros2-architecture/ with all 12 sections
- [ ] T033 [P] [US1] Create docs/chapters/chapter-03-ros2-workspace/ with all 12 sections
- [ ] T034 [P] [US1] Create docs/chapters/chapter-04-urdf-modeling/ with all 12 sections
- [ ] T035 [P] [US1] Create docs/chapters/chapter-05-ros2-control/ with all 12 sections
- [ ] T036 [P] [US1] Create docs/chapters/chapter-06-gazebo-simulation/ with all 12 sections
- [ ] T037 [P] [US1] Create docs/chapters/chapter-07-unity-simulation/ with all 12 sections
- [ ] T038 [P] [US1] Create docs/chapters/chapter-08-isaac-sim-intro/ with all 12 sections
- [ ] T039 [P] [US1] Create docs/chapters/chapter-09-isaac-manipulation/ with all 12 sections
- [ ] T040 [P] [US1] Create docs/chapters/chapter-10-isaac-locomotion/ with all 12 sections
- [ ] T041 [P] [US1] Create docs/chapters/chapter-11-vla-models/ with all 12 sections
- [ ] T042 [P] [US1] Create docs/chapters/chapter-12-vla-training/ with all 12 sections
- [ ] T043 [P] [US1] Create docs/chapters/chapter-13-vla-deployment/ with all 12 sections
- [ ] T044 [US1] Update sidebars.js to include all 13 chapters with proper categorization
- [ ] T045 [US1] Create docs/intro.md as homepage with course overview and navigation
- [ ] T046 [P] [US1] Add responsive CSS breakpoints in src/css/custom.css for mobile (375px+)
- [ ] T047 [P] [US1] Configure light/dark theme support in docusaurus.config.js
- [ ] T048 [US1] Test search functionality across all chapters (verify <1s response time)
- [ ] T049 [US1] Test responsive design on mobile devices (375px, 768px, 1024px widths)
- [ ] T050 [US1] Run content validation script to verify all chapters have 12 sections
- [ ] T051 [US1] Build production site with npm run build and verify <2s page load

**Checkpoint**: At this point, User Story 1 should be fully functional - all 13 chapters accessible with search and responsive design

---

## Phase 4: User Story 2 - Complete Hands-On Labs and Exercises (Priority: P2)

**Goal**: Add executable code examples, interactive exercises with difficulty tiers, and hardware tier documentation

**Independent Test**: Select any chapter's hands-on lab, follow instructions, execute code, verify expected output. Attempt exercises at all difficulty levels.

### Implementation for User Story 2

- [ ] T052 [P] [US2] Create src/components/CodeBlock/ component for syntax-highlighted code examples
- [ ] T053 [P] [US2] Create src/components/ExerciseCard/ component for tiered exercises (beginner/intermediate/advanced)
- [ ] T054 [P] [US2] Create src/components/HardwareTierSelector/ component to display hardware requirements
- [ ] T055 [US2] Enhance Chapter 1 hands-on-lab.md with step-by-step instructions, dependencies, expected outputs
- [ ] T056 [US2] Add executable code examples to Chapter 1 code-examples.md with comments
- [ ] T057 [US2] Add 3 tiered exercises to Chapter 1 exercises.md (beginner, intermediate, advanced)
- [ ] T058 [US2] Add hardware tier specifications to Chapter 1 hardware-requirements.md (minimum/recommended/premium)
- [ ] T059 [P] [US2] Enhance Chapter 2-5 hands-on labs with detailed instructions and troubleshooting
- [ ] T060 [P] [US2] Add executable code examples to Chapters 2-5 with ROS 2 code
- [ ] T061 [P] [US2] Add tiered exercises to Chapters 2-5
- [ ] T062 [P] [US2] Enhance Chapter 6-7 hands-on labs for simulation (Gazebo/Unity)
- [ ] T063 [P] [US2] Add simulation code examples to Chapters 6-7
- [ ] T064 [P] [US2] Add tiered exercises to Chapters 6-7
- [ ] T065 [P] [US2] Enhance Chapter 8-10 hands-on labs for NVIDIA Isaac
- [ ] T066 [P] [US2] Add Isaac Sim code examples to Chapters 8-10
- [ ] T067 [P] [US2] Add tiered exercises to Chapters 8-10
- [ ] T068 [P] [US2] Enhance Chapter 11-13 hands-on labs for VLA models
- [ ] T069 [P] [US2] Add VLA training/deployment code examples to Chapters 11-13
- [ ] T070 [P] [US2] Add tiered exercises to Chapters 11-13
- [ ] T071 [US2] Create .github/workflows/validate-code.yml to validate code examples execute successfully
- [ ] T072 [US2] Add hints and solutions to all exercises across all chapters
- [ ] T073 [US2] Test code examples on minimum hardware tier (cloud instances)
- [ ] T074 [US2] Document troubleshooting guidance for common errors in each lab

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - content accessible with executable labs and exercises

---

## Phase 5: User Story 3 - Get Contextual Help from AI Assistant (Priority: P3)

**Goal**: Embed RAG chatbot for contextual help with <3s response time, conversation history, and content-grounded responses

**Independent Test**: Open AI assistant, ask questions about chapter content, select text and request explanations, verify responses cite textbook sections within 3 seconds

### Implementation for User Story 3

- [ ] T075 [P] [US3] Implement backend/src/models/embedding.py for ContentEmbedding model
- [ ] T076 [P] [US3] Implement backend/src/services/vector_service.py for Qdrant vector operations
- [ ] T077 [P] [US3] Implement backend/src/services/content_indexer.py to generate embeddings from Markdown content
- [ ] T078 [US3] Create backend/src/scripts/index_content.py to index all 13 chapters into Qdrant
- [ ] T079 [P] [US3] Implement backend/src/services/chatbot_service.py with OpenAI integration and RAG logic
- [ ] T080 [P] [US3] Implement backend/src/api/chatbot.py with POST /v1/chat endpoint per chatbot-api.yaml
- [ ] T081 [P] [US3] Implement GET /v1/chat/history endpoint in backend/src/api/chatbot.py
- [ ] T082 [P] [US3] Implement POST /v1/chat/context endpoint in backend/src/api/chatbot.py
- [ ] T083 [US3] Add chatbot routes to backend/src/main.py
- [ ] T084 [P] [US3] Create src/components/ChatbotWidget/ React component with chat interface
- [ ] T085 [P] [US3] Implement text selection handler in ChatbotWidget for "Ask AI about this" feature
- [ ] T086 [P] [US3] Implement conversation history persistence in ChatbotWidget
- [ ] T087 [US3] Integrate ChatbotWidget into Docusaurus theme in src/theme/Root.js
- [ ] T088 [US3] Configure CORS in backend/src/main.py to allow frontend origin
- [ ] T089 [US3] Implement rate limiting in backend/src/api/chatbot.py to prevent API abuse
- [ ] T090 [US3] Add error handling and graceful degradation when AI assistant unavailable
- [ ] T091 [US3] Test chatbot response time (<3s for 95% of queries)
- [ ] T092 [US3] Test content grounding (responses cite relevant chapter sections)
- [ ] T093 [US3] Test conversation history persistence across sessions
- [ ] T094 [US3] Deploy backend API to Vercel with environment variables configured

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work - content, labs, and AI assistant functional

---

## Phase 6: User Story 4 - Track Learning Progress and Quiz Performance (Priority: P4)

**Goal**: Track chapter completion, quiz scores, and provide progress dashboard with recommendations

**Independent Test**: Complete quizzes for multiple chapters, view progress dashboard, verify quiz scores saved and chapters marked as completed

### Implementation for User Story 4

- [ ] T095 [P] [US4] Implement backend/src/models/progress.py with UserProgress model
- [ ] T096 [P] [US4] Implement backend/src/models/quiz.py with Quiz and QuizAttempt models
- [ ] T097 [P] [US4] Implement backend/src/services/progress_service.py for progress tracking logic
- [ ] T098 [P] [US4] Implement backend/src/services/quiz_service.py for quiz grading logic
- [ ] T099 [P] [US4] Implement backend/src/api/progress.py with GET /v1/progress endpoint per progress-api.yaml
- [ ] T100 [P] [US4] Implement POST /v1/progress/chapter endpoint in backend/src/api/progress.py
- [ ] T101 [P] [US4] Implement POST /v1/quiz/submit endpoint in backend/src/api/progress.py
- [ ] T102 [P] [US4] Implement GET /v1/quiz/attempts endpoint in backend/src/api/progress.py
- [ ] T103 [P] [US4] Implement GET /v1/stats endpoint in backend/src/api/progress.py
- [ ] T104 [US4] Add progress routes to backend/src/main.py
- [ ] T105 [P] [US4] Create src/components/QuizComponent/ React component for interactive quizzes
- [ ] T106 [P] [US4] Implement quiz grading and feedback display in QuizComponent
- [ ] T107 [P] [US4] Create src/components/ProgressTracker/ React component for chapter completion tracking
- [ ] T108 [P] [US4] Create src/pages/progress.tsx for progress dashboard page
- [ ] T109 [US4] Integrate QuizComponent into chapter quiz.md files across all chapters
- [ ] T110 [US4] Integrate ProgressTracker into Docusaurus navigation sidebar
- [ ] T111 [US4] Create quiz content (5-10 questions) for Chapter 1 quiz.md
- [ ] T112 [P] [US4] Create quiz content for Chapters 2-5 (ROS 2 module)
- [ ] T113 [P] [US4] Create quiz content for Chapters 6-7 (Simulation module)
- [ ] T114 [P] [US4] Create quiz content for Chapters 8-10 (Isaac module)
- [ ] T115 [P] [US4] Create quiz content for Chapters 11-13 (VLA module)
- [ ] T116 [US4] Implement learning recommendations logic in backend/src/services/progress_service.py
- [ ] T117 [US4] Add optional user authentication for progress tracking (anonymous support)
- [ ] T118 [US4] Test quiz auto-grading with correct/incorrect answers
- [ ] T119 [US4] Test progress persistence across sessions
- [ ] T120 [US4] Test recommendations based on quiz performance

**Checkpoint**: At this point, User Stories 1-4 should all work - full platform with progress tracking

---

## Phase 7: User Story 5 - Access Content in Preferred Language (Priority: P5)

**Goal**: Add Urdu (Roman script) translations with language switcher maintaining all functionality

**Independent Test**: Switch language to Urdu, navigate chapters, verify all text translated (except code), test AI assistant in Urdu

### Implementation for User Story 5

- [ ] T121 [US5] Configure i18n plugin in docusaurus.config.js with English and Urdu locales
- [ ] T122 [US5] Create i18n/ur/docusaurus-plugin-content-docs/ directory structure
- [ ] T123 [P] [US5] Translate Chapter 1 content to Urdu in i18n/ur/docusaurus-plugin-content-docs/chapters/chapter-01-introduction/
- [ ] T124 [P] [US5] Translate Chapters 2-5 content to Urdu
- [ ] T125 [P] [US5] Translate Chapters 6-7 content to Urdu
- [ ] T126 [P] [US5] Translate Chapters 8-10 content to Urdu
- [ ] T127 [P] [US5] Translate Chapters 11-13 content to Urdu
- [ ] T128 [P] [US5] Translate UI elements (navigation, buttons, labels) to Urdu in i18n/ur/docusaurus-theme-classic/
- [ ] T129 [US5] Add language switcher component to Docusaurus navbar
- [ ] T130 [US5] Update backend/src/services/chatbot_service.py to detect and respond in Urdu
- [ ] T131 [US5] Index Urdu content embeddings in Qdrant for RAG in Urdu
- [ ] T132 [US5] Ensure code examples remain in English while explanatory text is in Urdu
- [ ] T133 [US5] Test language switching (<2s transition time)
- [ ] T134 [US5] Test AI assistant responses in Urdu
- [ ] T135 [US5] Test that current position preserved when switching languages

**Checkpoint**: At this point, all 5 user stories complete - full multilingual platform

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T136 [P] Optimize images and diagrams for fast loading (WebP format, compression)
- [ ] T137 [P] Implement lazy loading for Mermaid diagrams
- [ ] T138 [P] Add meta tags and SEO optimization in docusaurus.config.js
- [ ] T139 [P] Create README.md with project overview and setup instructions
- [ ] T140 [P] Create CONTRIBUTING.md with contribution guidelines
- [ ] T141 [P] Add analytics tracking (optional) in docusaurus.config.js
- [ ] T142 [P] Implement caching strategy for backend API responses
- [ ] T143 [P] Add monitoring and logging for backend API
- [ ] T144 [P] Create backend/Dockerfile for containerized deployment
- [ ] T145 [P] Add security headers in backend/src/main.py
- [ ] T146 [US1] Validate all internal links are not broken
- [ ] T147 [US1] Verify all diagrams render correctly in light and dark modes
- [ ] T148 [US2] Verify all code examples have proper syntax highlighting
- [ ] T149 [US3] Test AI assistant graceful degradation when OpenAI API unavailable
- [ ] T150 [US4] Verify quiz explanations are educational and not just answers
- [ ] T151 Run full content validation across all chapters
- [ ] T152 Run performance testing (page load <2s, search <1s, API <3s)
- [ ] T153 Run accessibility testing (WCAG 2.1 AA compliance)
- [ ] T154 Deploy to GitHub Pages and verify production deployment
- [ ] T155 Verify 99%+ uptime over 7-day monitoring period

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 content but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Requires US1 content for indexing but independently testable
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Requires US1 content and US2 quizzes but independently testable
- **User Story 5 (P5)**: Can start after US1 complete - Requires content to translate

### Within Each User Story

- Models before services
- Services before API endpoints
- Backend before frontend integration
- Core implementation before testing
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Stories 1-4 can start in parallel (US5 depends on US1)
- Within each user story, tasks marked [P] can run in parallel
- Chapter content creation (T020-T043) can be parallelized across multiple contributors
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1 (Content Creation)

```bash
# Launch all Chapter 1 sections together:
Task: "Create docs/chapters/chapter-01-introduction/index.md"
Task: "Create docs/chapters/chapter-01-introduction/learning-objectives.md"
Task: "Create docs/chapters/chapter-01-introduction/prerequisites.md"
# ... (all 12 sections)

# Launch multiple chapters together:
Task: "Create docs/chapters/chapter-02-ros2-architecture/ with all 12 sections"
Task: "Create docs/chapters/chapter-03-ros2-workspace/ with all 12 sections"
Task: "Create docs/chapters/chapter-04-urdf-modeling/ with all 12 sections"
```

---

## Parallel Example: User Story 3 (AI Assistant)

```bash
# Launch backend models and services together:
Task: "Implement backend/src/models/embedding.py"
Task: "Implement backend/src/services/vector_service.py"
Task: "Implement backend/src/services/content_indexer.py"

# Launch API endpoints together:
Task: "Implement POST /v1/chat endpoint"
Task: "Implement GET /v1/chat/history endpoint"
Task: "Implement POST /v1/chat/context endpoint"

# Launch frontend components together:
Task: "Create src/components/ChatbotWidget/"
Task: "Implement text selection handler"
Task: "Implement conversation history persistence"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T019) - CRITICAL
3. Complete Phase 3: User Story 1 (T020-T051)
4. **STOP and VALIDATE**: Test all 13 chapters accessible, search works, responsive design
5. Deploy to GitHub Pages and demo

**Result**: Functional textbook with all content, search, and responsive design

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (MVP + Labs)
4. Add User Story 3 → Test independently → Deploy/Demo (MVP + Labs + AI)
5. Add User Story 4 → Test independently → Deploy/Demo (MVP + Labs + AI + Progress)
6. Add User Story 5 → Test independently → Deploy/Demo (Full Platform)
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T019)
2. Once Foundational is done:
   - **Team A**: User Story 1 - Content (T020-T051)
   - **Team B**: User Story 2 - Labs (T052-T074) - starts after US1 content available
   - **Team C**: User Story 3 - AI Assistant (T075-T094) - can start in parallel
   - **Team D**: User Story 4 - Progress (T095-T120) - can start in parallel
3. User Story 5 starts after US1 complete
4. Stories complete and integrate independently

---

## Task Summary

- **Total Tasks**: 155
- **Phase 1 (Setup)**: 10 tasks
- **Phase 2 (Foundational)**: 9 tasks (BLOCKING)
- **Phase 3 (US1 - Content)**: 32 tasks
- **Phase 4 (US2 - Labs)**: 23 tasks
- **Phase 5 (US3 - AI Assistant)**: 20 tasks
- **Phase 6 (US4 - Progress)**: 26 tasks
- **Phase 7 (US5 - Multilingual)**: 15 tasks
- **Phase 8 (Polish)**: 20 tasks

**Parallelizable Tasks**: 98 tasks marked with [P]

**MVP Scope** (Recommended): Phase 1 + Phase 2 + Phase 3 (User Story 1 only) = 51 tasks

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Tests are OPTIONAL - not included as spec doesn't explicitly request TDD
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Content creation (Chapters 1-13) is the most time-intensive part
- Backend API can be developed in parallel with frontend content
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
