---
id: 001
title: Physical AI Robotics Textbook Constitution
stage: constitution
date: 2026-01-11
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: HAJI LAPTOP KARACHI
command: /sp.constitution
labels: ["constitution", "project-setup", "governance", "educational-content"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (constitution document)
---

## Prompt

# PROMPT: Physical AI & Humanoid Robotics Textbook

## OBJECTIVE
Create an AI-native textbook using Docusaurus, deployed to GitHub Pages, teaching a 13-week Physical AI & Humanoid Robotics course.

## TECH STACK
- **Frontend**: Docusaurus (latest)
- **Deployment**: GitHub Pages
- **Backend**: FastAPI + OpenAI ChatKit SDK
- **Database**: Neon Postgres + Qdrant Cloud
- **Development**: Spec-Kit Plus methodology

## COURSE STRUCTURE (13 Chapters across 4 Modules)

### Module 1: ROS 2 (Weeks 1-5)
1. Introduction to Physical AI & Embodied Intelligence
2. ROS 2 Architecture & Core Concepts
3. Building with ROS 2 (Python/rclpy)
4. URDF & Robot Modeling

### Module 2: Gazebo & Unity (Weeks 6-7)
5. Physics Simulation with Gazebo
6. High-Fidelity Rendering with Unity

### Module 3: NVIDIA Isaac (Weeks 8-10)
7. Isaac Platform & Photorealistic Simulation
8. AI-Powered Perception & Navigation (VSLAM, Nav2)
9. Humanoid Dynamics & Control

### Module 4: Vision-Language-Action (Weeks 11-13)
10. Natural Human-Robot Interaction
11. Voice-to-Action (OpenAI Whisper)
12. Cognitive Planning with LLMs
13. Capstone: Autonomous Humanoid Project

## CHAPTER TEMPLATE (Each Chapter Must Include)
- Overview
- Learning Objectives (4-5 points)
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

## CORE REQUIREMENTS (100 points)
1. Complete 13-chapter Docusaurus book
2. Embedded RAG chatbot:
   - Answer questions about content
   - Support user-selected text queries
   - FastAPI backend + ChatKit SDK
   - Neon Postgres + Qdrant storage

## BONUS FEATURES (50 points each)
1. **Reusable Intelligence**: Claude Code Subagents + Agent Skills
2. **Better-Auth**: Signup/signin collecting user software/hardware background
3. **Personalization**: Button to adjust content based on user level
4. **Urdu Translation**: Button to translate chapters to Roman Urdu

## HARDWARE CONSIDERATIONS
Include in content:
- Minimum: Cloud instances
- Recommended: RTX 4070 Ti + Jetson Orin Nano
- Premium: RTX 4090 + Jetson Orin NX + Unitree G1

## CODE STANDARDS
- Ubuntu 22.04 LTS compatible
- ROS 2 Humble/Iron
- Production-ready with error handling
- Well-commented and documented

## DELIVERABLES
- Public GitHub repo
- Live deployment URL (GitHub Pages)
- 90-second demo video
- All chapters complete with template sections
- Working RAG chatbot
- (Optional) Bonus features implemented

## SUCCESS CRITERIA
- Technical accuracy
- Beginner-friendly progression
- Functional hands-on labs
- Smooth RAG chatbot integration
- Professional UX/UI

## Response snapshot

Created comprehensive constitution (v1.0.0) for Physical AI & Humanoid Robotics Textbook project with 7 core principles:

**Core Principles:**
1. Technical Accuracy & Educational Rigor - Production-ready code, verifiable claims, Ubuntu 22.04 + ROS 2 compatibility
2. Structured Learning Progression - Mandatory 12-section chapter template for consistency
3. Beginner-Friendly Progression (NON-NEGOTIABLE) - Module 1-4 progression from ROS 2 fundamentals to VLA models
4. Hands-On Lab Quality - Executable, self-contained, error-handled, validated labs
5. Deployment & Accessibility - GitHub Pages deployment, <2s page load, responsive design, multilingual support
6. RAG Chatbot Integration - Accurate responses, context support, Neon Postgres + Qdrant, <3s response time
7. Code Standards & Compatibility - Ubuntu 22.04 LTS, ROS 2 Humble/Iron, production-ready with error handling

**Additional Sections:**
- Content Standards: Chapter completeness, hardware guidance (min/recommended/premium), quiz quality
- Development Workflow: 6-step content creation process, code example standards, bonus feature guidelines
- Governance: Amendment process, semantic versioning, compliance review requirements

**Template Alignment:**
- spec-template.md: Aligned with user story prioritization
- plan-template.md: Aligned with technical context requirements
- tasks-template.md: Aligned with phase-based implementation
- README.md: May need minor updates to reference constitution

## Outcome

- ✅ Impact: Established foundational governance for educational robotics textbook project with clear quality standards and compliance requirements
- 🧪 Tests: N/A (governance document)
- 📁 Files: Created .specify/memory/constitution.md (v1.0.0)
- 🔁 Next prompts: Feature specification for individual chapters or RAG chatbot implementation
- 🧠 Reflection: Constitution balances educational rigor with practical implementation constraints. Non-negotiable beginner-friendly progression principle ensures pedagogical soundness. Hardware tier guidance (min/recommended/premium) addresses diverse student resources.

## Evaluation notes (flywheel)

- Failure modes observed: None - initial constitution creation
- Graders run and results (PASS/FAIL): N/A (no automated grading for constitution)
- Prompt variant (if applicable): N/A (initial version)
- Next experiment (smallest change to try): Monitor chapter creation process to validate whether 12-section template is sufficient or needs refinement based on actual content development
