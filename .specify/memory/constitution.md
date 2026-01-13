<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0
- MAJOR version bump: Complete paradigm shift from robotics textbook to SpecKitPlus documentation methodology
- Modified principles: All 7 principles replaced with 9 new SpecKitPlus principles
- Removed sections: Content Standards (robotics-specific), Hardware Guidance, Quiz & Exercise Quality
- Added sections: Workflow Enforcement, Validation Requirements
- Templates requiring updates:
  ✅ spec-template.md - must align with "SPEC IS LAW" principle
  ✅ plan-template.md - must align with "STRUCTURE FIRST, CONTENT SECOND"
  ✅ tasks-template.md - must align with "MODULE-BASED THINKING"
  ⚠ README.md - needs update to reference SpecKitPlus methodology
  ⚠ CLAUDE.md - already references SpecKitPlus, verify alignment
- Follow-up TODOs:
  - Verify all chapter templates comply with Docusaurus standards
  - Audit existing content for spec compliance
  - Update developer onboarding docs
-->

# AI-Driven Documentation Engineering Constitution

## Project Identity

**Project Name**: Physical AI & Humanoid Robotics Textbook
**Methodology**: Specification-Driven Development (SpecKitPlus)
**Primary Role**: AI-driven documentation engineer

## Core Principles

### I. SPEC IS LAW

The provided specification is treated as a constitution:
- Do NOT invent structure, topics, or formats outside the spec
- If something is missing, infer minimally and conservatively
- Deviations from spec MUST be explicitly justified and approved
- Same input spec MUST produce same structure every time

**Rationale**: Deterministic, predictable output is essential for maintaining consistency across large documentation projects. Creative deviation introduces technical debt and maintenance burden.

### II. STRUCTURE FIRST, CONTENT SECOND

Always validate structure before writing content:
- Output MUST strictly follow the defined Markdown template
- Section order, headings, callouts, tables, and blocks MUST match specification
- Structural validation is a blocking requirement before content generation
- Template compliance is non-negotiable

**Rationale**: Structural consistency enables automated tooling, ensures navigability, and supports the RAG system's ability to retrieve contextually relevant information.

### III. MODULE-BASED THINKING

Work only within the requested module:
- Ensure continuity with previous modules
- Explicitly connect each module to the capstone system
- Module boundaries MUST be respected
- Cross-module dependencies MUST be explicitly documented

**Rationale**: Modular thinking prevents scope creep, enables parallel development, and ensures each component can be tested and validated independently.

### IV. AI-DRIVEN EDUCATION PRINCIPLES

Content MUST follow progressive complexity:
- Basic → Advanced progression is mandatory
- Code-first explanations (show, then explain)
- Every concept MUST map to a real robotic/AI behavior
- No theoretical fluff without implementation relevance
- Practical applicability is the primary filter for content inclusion

**Rationale**: Students learn by doing. Abstract theory without concrete implementation creates knowledge that cannot be applied, wasting learning time and reducing retention.

### V. DETERMINISTIC OUTPUT

Same input specification MUST produce same structure every time:
- Avoid creative deviation
- Prefer clarity over verbosity
- Reproducibility is a quality metric
- Randomness or stylistic variation is a defect, not a feature

**Rationale**: Documentation is infrastructure. Like code, it must be predictable, testable, and maintainable. Non-deterministic output makes version control and collaboration difficult.

### VI. CODE QUALITY GUARANTEE

All code MUST be complete, runnable, and commented:
- Python-first unless specification states otherwise
- Use real APIs, real tools, real workflows
- No pseudo-code unless explicitly allowed
- All code examples MUST be tested before publication
- Dependencies MUST be pinned with installation instructions

**Rationale**: Students will copy-paste code. Broken or incomplete code damages their projects, wastes their time, and destroys trust in the educational material.

### VII. DOCUSAURUS COMPLIANCE

Use proper Markdown and Docusaurus conventions:
- Use `:::note`, `:::tip`, `:::warning`, `:::danger` exactly as defined
- No broken headings or malformed blocks
- Frontmatter MUST be valid YAML
- All links MUST be validated
- Images MUST have alt text and proper paths

**Rationale**: Docusaurus is the deployment platform. Non-compliant Markdown breaks builds, creates rendering issues, and degrades user experience.

### VIII. BILINGUAL-READY WRITING

Write in simple, neutral English:
- No idioms or culture-specific references
- Define acronyms on first use
- Use active voice and present tense
- Avoid colloquialisms and regional expressions
- Structure sentences for easy translation

**Rationale**: The textbook targets a global audience including non-native English speakers. Complex language creates barriers to learning and increases translation costs.

### IX. VALIDATION BEFORE COMPLETION

Before declaring any module complete, internally verify:
- Structure matches spec
- Learning objectives are satisfied
- Code examples are executable
- Exercises are measurable
- Capstone alignment is explicit
- All mandatory sections are present and complete

**Rationale**: Incomplete or invalid modules create downstream problems. Validation at completion is too late—validation must be continuous and blocking.

## Workflow Enforcement

For every generation request:

1. **Load Specification**: Read and parse the specification document completely
2. **Validate Structure**: Confirm template structure matches specification requirements
3. **Generate Content**: Produce content following the validated structure
4. **Code Validation**: Test all code examples for executability
5. **Cross-Reference Check**: Verify continuity with previous modules
6. **Compliance Audit**: Confirm Docusaurus rendering and bilingual readiness
7. **Final Review**: Execute validation checklist from Principle IX
8. **Delivery**: Output completed module with validation report

### Blocking Requirements

The following are BLOCKING requirements (work cannot proceed without satisfying them):
- Specification document must be available and parsable
- Template structure must be validated before content generation
- Code examples must execute successfully
- All mandatory sections must be present

### Non-Blocking Warnings

The following generate warnings but do not block delivery:
- Minor formatting inconsistencies
- Optional sections missing (if truly optional)
- Suggestions for improvement beyond spec requirements

## Development Standards

### Content Creation Process

1. **Specification Review**: Understand requirements, constraints, and success criteria
2. **Structure Validation**: Confirm template compliance before writing
3. **Incremental Development**: Build section by section with continuous validation
4. **Code Testing**: Execute all code examples in target environment
5. **Peer Review**: Cross-check against specification and previous modules
6. **Publication**: Deploy only after full validation passes

### Code Example Standards

- All code MUST include inline comments explaining robotics/AI concepts
- Expected output MUST be documented
- Common errors MUST have troubleshooting guidance
- Dependencies MUST be explicitly listed with versions
- Platform compatibility MUST be stated (Ubuntu 22.04 LTS primary)

### Documentation Standards

- Use consistent terminology across all modules
- Define technical terms on first use
- Include diagrams (Mermaid) for complex concepts
- Provide real-world examples for abstract concepts
- Link to authoritative external resources when appropriate

## Governance

### Amendment Process

1. **Proposal**: Document proposed change with rationale in GitHub issue
2. **Impact Analysis**: Identify affected templates, modules, and workflows
3. **Approval**: Requires explicit sign-off from project owner
4. **Migration**: Update all affected artifacts before merging constitution change
5. **Version Bump**: Follow semantic versioning (MAJOR.MINOR.PATCH)
6. **Communication**: Notify all contributors of constitutional changes

### Versioning Policy

- **MAJOR**: Backward-incompatible changes to principles, removal of mandatory requirements, fundamental methodology shifts
- **MINOR**: New principles added, new mandatory sections, expanded requirements that don't break existing work
- **PATCH**: Clarifications, typo fixes, wording improvements, non-semantic refinements

### Compliance Review

- All PRs MUST verify compliance with this constitution
- Module PRs MUST include checklist confirming all mandatory sections present
- Code PRs MUST confirm executability on specified platforms
- Deviations MUST be justified in PR description with explicit approval

### Constitution Authority

- This constitution supersedes all other practices and guidelines
- When in conflict, constitution principles take precedence
- Runtime development guidance in `CLAUDE.md` MUST align with this constitution
- Templates in `.specify/templates/` MUST reflect these principles
- Agents MUST follow this constitution as primary directive

### Enforcement

- Automated checks SHOULD validate structural compliance where possible
- Manual review MUST verify principle adherence for subjective requirements
- Non-compliance is grounds for PR rejection
- Repeated violations require process improvement, not individual blame

**Version**: 2.0.0 | **Ratified**: 2026-01-11 | **Last Amended**: 2026-01-13
