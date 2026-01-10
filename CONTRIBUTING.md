# Content Authoring Guide

This guide explains how to create high-quality chapters for the Physical AI & Humanoid Robotics text-book that meet our constitution's quality standards.

## Table of Contents

- [Chapter Structure Requirements](#chapter-structure-requirements)
- [Creating a New Chapter](#creating-a-new-chapter)
- [Writing Guidelines](#writing-guidelines)
- [Mermaid Diagrams](#mermaid-diagrams)
- [Code Examples](#code-examples)
- [Translation Workflow](#translation-workflow)
- [Quality Checklist](#quality-checklist)

## Chapter Structure Requirements

Every chapter **MUST** include these 8 elements (per project constitution):

1. **Learning Objectives** - Measurable goals using Bloom's taxonomy
2. **Concepts** - Clear explanations with Mermaid diagrams
3. **Hands-on Lab** - Tested, runnable code with step-by-step instructions
4. **Pitfalls** - Common mistakes and how to avoid them
5. **Exercises** - Problems at 3 levels (beginner, intermediate, advanced)
6. **Quiz** - Knowledge check questions with explanations
7. **FAQs** - Frequently asked questions in RAG-ready format
8. **Tested Code** - All code examples must be verified to run

## Creating a New Chapter

### Step 1: Create Directory Structure

```bash
# Create chapter directory
mkdir -p docs/chapters/chapter-XX-topic-name

# Create required files
cd docs/chapters/chapter-XX-topic-name
touch index.md
touch learning-objectives.md
touch concepts.md
touch hands-on-lab.md
touch exercises.md
touch quiz.md
touch faqs.md
```

### Step 2: Chapter Index (index.md)

```markdown
---
id: chapter-XX-topic-name
title: "Chapter XX: Topic Name"
sidebar_label: "Chapter XX: Topic"
sidebar_position: XX
description: "Brief description of what this chapter covers"
tags: [tag1, tag2, tag3]
---

# Chapter XX: Topic Name

## Overview

Brief introduction to the chapter topic (2-3 paragraphs).

## What You'll Learn

Summary of learning objectives (bullet list).

## Chapter Sections

- [Learning Objectives](./learning-objectives.md)
- [Core Concepts](./concepts.md)
- [Hands-on Lab](./hands-on-lab.md)
- [Exercises](./exercises.md)
- [Quiz](./quiz.md)
- [FAQs](./faqs.md)

## Prerequisites

- Prerequisite 1
- Prerequisite 2

## Estimated Time

- Reading: XX minutes
- Hands-on Lab: XX minutes
- Exercises: XX minutes
- Total: XX minutes
```

### Step 3: Learning Objectives (learning-objectives.md)

Use Bloom's taxonomy verbs (Remember, Understand, Apply, Analyze, Evaluate, Create):

```markdown
---
sidebar_position: 1
---

# Learning Objectives

By the end of this chapter, you will be able to:

1. **Define** [concept] and explain its significance in [context]
2. **Identify** the key components of [system/process]
3. **Describe** how [process] works and its applications
4. **Analyze** [scenario] to determine appropriate [solution]
5. **Discuss** the ethical implications of [topic]

Each objective should be:
- **Measurable** - Can be tested/verified
- **Specific** - Clear and unambiguous
- **Action-oriented** - Uses Bloom's taxonomy verbs
```

### Step 4: Concepts (concepts.md)

Include at least one Mermaid diagram:

```markdown
---
sidebar_position: 2
---

# Core Concepts

## Concept 1: [Name]

Explanation of the concept (2-3 paragraphs).

### Visual Representation

\`\`\`mermaid
graph TD
    A[Start] --> B[Process]
    B --> C[Decision]
    C -->|Yes| D[Outcome 1]
    C -->|No| E[Outcome 2]
\`\`\`

### Key Points

- Point 1
- Point 2
- Point 3

### Common Pitfalls ⚠️

- **Pitfall 1**: Description and how to avoid it
- **Pitfall 2**: Description and how to avoid it

## Concept 2: [Name]

[Continue with additional concepts...]
```

### Step 5: Hands-on Lab (hands-on-lab.md)

Provide complete, tested code:

```markdown
---
sidebar_position: 3
---

# Hands-on Lab: [Lab Title]

## Objective

What you'll build and learn in this lab.

## Prerequisites

- Python 3.10+
- Required libraries: `pip install library1 library2`

## Step 1: [Setup/First Step]

Explanation of what we're doing.

\`\`\`python
# Complete, runnable code
import library

def function_name():
    """Docstring explaining the function."""
    # Implementation
    pass

# Example usage
if __name__ == "__main__":
    result = function_name()
    print(f"Result: {result}")
\`\`\`

**Explanation:**
- Line-by-line explanation of key parts
- Why we're doing it this way

## Step 2: [Next Step]

[Continue with additional steps...]

## Expected Output

\`\`\`
Expected output when running the code
\`\`\`

## Challenges

Try these extensions:
1. Modify the code to [variation]
2. Add [feature] to improve [aspect]
3. Experiment with [parameter] and observe changes

## Troubleshooting

**Issue**: Common error message
**Solution**: How to fix it

## Complete Code

\`\`\`python
# Full working code in one place for easy copying
[Complete implementation]
\`\`\`
```

### Step 6: Exercises (exercises.md)

Three difficulty levels with answers:

```markdown
---
sidebar_position: 4
---

# Exercises

## Beginner Level 🟢

### Exercise 1: [Title]

**Problem**: Clear problem statement

**Hints**:
- Hint 1
- Hint 2

<details>
<summary>Show Answer</summary>

**Solution**:
\`\`\`python
# Code solution
\`\`\`

**Explanation**: Why this solution works
</details>

## Intermediate Level 🟡

### Exercise 4: [Title]

**Problem**: More complex problem requiring multiple concepts

**Requirements**:
- Requirement 1
- Requirement 2

<details>
<summary>Show Answer</summary>

**Solution**:
\`\`\`python
# More complex solution
\`\`\`

**Explanation**: Detailed explanation of approach
</details>

## Advanced Level 🔴

### Exercise 7: [Title]

**Problem**: Challenging problem requiring synthesis of concepts

**Constraints**:
- Constraint 1
- Constraint 2

<details>
<summary>Show Answer</summary>

**Solution**:
\`\`\`python
# Advanced solution
\`\`\`

**Explanation**: Discussion of tradeoffs and design decisions
</details>
```

### Step 7: Quiz (quiz.md)

Mix of question types:

```markdown
---
sidebar_position: 5
---

# Knowledge Check Quiz

## Question 1: [Topic]

**Question**: What is [concept]?

**Options**:
- A) Option 1
- B) Option 2
- C) Option 3
- D) Option 4

<details>
<summary>Show Answer</summary>

**Answer**: C

**Explanation**: Detailed explanation of why C is correct and why others are wrong.
</details>

## Question 2: True/False

**Statement**: [Statement to evaluate]

<details>
<summary>Show Answer</summary>

**Answer**: True/False

**Explanation**: Why this is true/false with examples.
</details>

## Question 3: Short Answer

**Question**: Explain [concept] in your own words.

<details>
<summary>Sample Answer</summary>

**Answer**: A good answer should include:
- Point 1
- Point 2
- Point 3

**Example**: [Sample complete answer]
</details>
```

### Step 8: FAQs (faqs.md)

RAG-ready format:

```markdown
---
sidebar_position: 6
---

# Frequently Asked Questions

## General Questions

### What is [concept]?

[Concept] is [definition]. It is used for [purpose] and works by [mechanism].

**Key points**:
- Point 1
- Point 2

### How does [process] work?

[Process] works through these steps:
1. Step 1
2. Step 2
3. Step 3

### When should I use [technique]?

Use [technique] when:
- Scenario 1
- Scenario 2

Avoid using it when:
- Scenario 3
- Scenario 4

## Technical Questions

### Why does [error] occur?

This error occurs because [reason]. To fix it:
1. Solution step 1
2. Solution step 2

### What's the difference between [A] and [B]?

| Aspect | A | B |
|--------|---|---|
| Feature 1 | Value | Value |
| Feature 2 | Value | Value |
| Use case | When to use A | When to use B |

## Best Practices

### What are the best practices for [topic]?

1. **Practice 1**: Description and rationale
2. **Practice 2**: Description and rationale
3. **Practice 3**: Description and rationale

### How can I avoid common mistakes?

Common mistakes and solutions:
- **Mistake 1**: How to avoid it
- **Mistake 2**: How to avoid it
```

## Writing Guidelines

### Style and Tone

- **Clear and concise**: Avoid jargon; explain technical terms
- **Active voice**: "The model processes data" not "Data is processed by the model"
- **Present tense**: "The algorithm works" not "The algorithm worked"
- **Second person**: "You will learn" not "Students will learn"
- **Inclusive language**: Use "they/their" for singular pronouns

### Formatting

- **Headings**: Use hierarchical structure (H1 → H2 → H3)
- **Lists**: Use bullet points for unordered, numbers for sequential steps
- **Code blocks**: Always specify language for syntax highlighting
- **Emphasis**: Use **bold** for key terms, *italics* for emphasis
- **Links**: Use descriptive text, not "click here"

### Code Standards

- **Runnable**: All code must be tested and work as shown
- **Comments**: Explain why, not what (code shows what)
- **PEP 8**: Follow Python style guide
- **Type hints**: Use type annotations for clarity
- **Error handling**: Show proper exception handling
- **Examples**: Include realistic use cases

## Mermaid Diagrams

### Supported Diagram Types

```markdown
# Flowchart
\`\`\`mermaid
graph TD
    A[Start] --> B[Process]
    B --> C{Decision}
    C -->|Yes| D[End]
    C -->|No| B
\`\`\`

# Sequence Diagram
\`\`\`mermaid
sequenceDiagram
    User->>System: Request
    System->>Database: Query
    Database-->>System: Data
    System-->>User: Response
\`\`\`

# Class Diagram
\`\`\`mermaid
classDiagram
    class Animal {
        +String name
        +int age
        +makeSound()
    }
    class Dog {
        +bark()
    }
    Animal <|-- Dog
\`\`\`
```

### Diagram Best Practices

- Keep diagrams simple and focused
- Use consistent styling across chapters
- Add descriptive labels to all nodes
- Test diagrams render correctly before committing

## Translation Workflow

### Translating to Urdu

1. **Create directory structure**:
   ```bash
   mkdir -p i18n/ur/docusaurus-plugin-content-docs/current/chapters/chapter-XX-topic-name
   ```

2. **Translate content files**:
   - Translate all markdown content
   - Keep code examples in English (with Urdu comments if helpful)
   - Preserve all frontmatter (id, title, etc.)
   - Maintain Mermaid diagram structure (translate labels only)

3. **RTL considerations**:
   - Text automatically flows RTL
   - Code blocks remain LTR
   - Diagrams may need label adjustments
   - Test rendering in Urdu locale

4. **Translation quality**:
   - Use proper Urdu technical terminology
   - Maintain consistent terminology across chapters
   - Preserve formatting and structure
   - Test all links work in translated version

## Quality Checklist

Before submitting a chapter, verify:

### Content Completeness
- [ ] All 8 required elements present
- [ ] Learning objectives are measurable
- [ ] At least one Mermaid diagram in concepts
- [ ] Hands-on lab code is tested and runs
- [ ] Pitfalls section addresses common mistakes
- [ ] Exercises at all 3 difficulty levels
- [ ] Quiz has 5-10 questions with explanations
- [ ] FAQs in RAG-ready format (20+ Q&As)

### Code Quality
- [ ] All code examples tested and working
- [ ] Code follows PEP 8 style guide
- [ ] Type hints included where appropriate
- [ ] Error handling demonstrated
- [ ] Comments explain why, not what
- [ ] Dependencies clearly listed

### Writing Quality
- [ ] Clear, concise language
- [ ] Active voice and present tense
- [ ] Technical terms explained
- [ ] Proper heading hierarchy
- [ ] No spelling or grammar errors
- [ ] Links work correctly

### Formatting
- [ ] Frontmatter complete and correct
- [ ] Code blocks have language specified
- [ ] Mermaid diagrams render correctly
- [ ] Images have alt text
- [ ] Tables formatted properly
- [ ] Collapsible sections work

### Accessibility
- [ ] Headings in logical order
- [ ] Alt text for all images
- [ ] Color not sole means of conveying info
- [ ] Code examples have text descriptions
- [ ] Links have descriptive text

### SEO
- [ ] Descriptive title and description
- [ ] Relevant tags included
- [ ] Keywords naturally integrated
- [ ] Internal links to related chapters
- [ ] Clear URL structure

## Testing Your Chapter

### Local Testing

```bash
# Start development server
npm start

# Test in browser
# - Navigate to your chapter
# - Verify all sections load
# - Test all links
# - Check Mermaid diagrams render
# - Test code examples (copy/paste/run)
# - Verify responsive design (mobile view)
# - Test dark mode
```

### Translation Testing

```bash
# Start with Urdu locale
npm start -- --locale ur

# Verify:
# - RTL layout works correctly
# - Urdu text renders properly
# - Code blocks remain LTR
# - Navigation works
# - Language switcher toggles correctly
```

## Getting Help

- **Documentation**: See [README.md](./README.md) for setup
- **Deployment**: See [docs/deployment-guide.md](./docs/deployment-guide.md)
- **Constitution**: See [.specify/memory/constitution.md](./.specify/memory/constitution.md) for principles
- **Examples**: Reference Chapter 1 as a template

## Chapter Template

Use Chapter 1 as a reference template:
- `docs/chapters/chapter-01-introduction/` - Complete example
- Copy structure and adapt for your topic
- Maintain consistent formatting and style
- Follow all 8 required elements

---

**Remember**: Quality over quantity. A well-crafted chapter with tested code and clear explanations is more valuable than rushing through content. Take time to verify everything works before submitting.
