---
id: 001
title: Create AI Textbook Platform Constitution
stage: constitution
date: 2026-01-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: "[firstname"
command: /sp.constitution
labels: ["constitution", "hackathon", "project-setup", "spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
Mission:
Build a spec-driven, production-grade AI textbook platform using Docusaurus + GitHub Pages, authored via Spec-Kit Plus + Claude Code, with an embedded RAG chatbot, auth, personalization, and Urdu translation, maximizing hackathon score (100 + 200 bonus).

Core Rules (Non-Negotiable):

Spec-first, reuse-first

Runnable code only (Ubuntu 22.04, Python 3.10+)

No hallucinated APIs

RAG answers require retrieval

Selected-text Q&A must use ONLY selected text

Prefer subagents & skills over monolithic generation

Book Requirements (Every Chapter):

Learning objectives

Clear concepts + Mermaid diagrams

Tested code + hands-on lab

Pitfalls

Exercises (beginner/intermediate/advanced)

Quiz

FAQs (RAG-ready)

RAG Stack (Mandatory):
OpenAI Agents/ChatKit · FastAPI · Neon Serverless Postgres · Qdrant Cloud
No retrieval → no answer.

Auth & Personalization (Bonus):

Auth via Better-Auth

Collect user background at signup

Chapter-level "Personalize Content" button

Deterministic, reversible adaptation
```

## Response snapshot

Created the project constitution v1.0.0 with six core principles:

1. **Spec-First Development**: Mandatory spec-driven workflow, reuse-first approach
2. **Runnable Code Standards**: Ubuntu 22.04, Python 3.10+, no pseudo-code
3. **RAG Integrity & No Hallucination**: Retrieval-grounded answers only, no API hallucination
4. **Subagent & Skills Architecture**: Modular agents over monolithic generation
5. **Chapter Quality Standards**: 8 mandatory elements per chapter (objectives, concepts, diagrams, code, labs, pitfalls, exercises, quiz, FAQs)
6. **Technology Stack Compliance**: Mandatory RAG stack (OpenAI Agents, FastAPI, Neon Postgres, Qdrant) + bonus auth stack (Better-Auth)

Added three major sections:
- **Book Requirements**: Pedagogical standards for all content
- **RAG Stack Requirements**: 5 mandatory implementation rules
- **Auth & Personalization**: 4 bonus feature requirements

Constitution file created at `.specify/memory/constitution.md` with sync impact report embedded.

## Outcome

- ✅ Impact: Established foundational governance for hackathon project with clear scoring alignment (100 base + 200 bonus points)
- 🧪 Tests: No tests created (constitution is governance document)
- 📁 Files: Created/updated `.specify/memory/constitution.md` (130 lines)
- 🔁 Next prompts: Review dependent templates (plan-template.md, spec-template.md, tasks-template.md) for alignment; create first feature spec using /sp.specify
- 🧠 Reflection: Constitution successfully captures all hackathon requirements as enforceable principles. Technology stack is concrete and testable. Chapter quality standards are comprehensive and measurable.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution creation successful on first attempt
- Graders run and results (PASS/FAIL): Manual validation PASS (no unresolved placeholders, all sections complete, version semantics correct)
- Prompt variant (if applicable): N/A (initial constitution)
- Next experiment (smallest change to try): Validate constitution principles against actual feature implementation workflow to identify gaps or ambiguities
