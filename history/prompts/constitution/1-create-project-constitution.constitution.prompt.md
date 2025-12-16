---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: smc
command: /sp.constitution
labels: [constitution,setup,initial]
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

```
/sp.constitution
Project: AI/Spec-Driven Book with Embedded RAG Chatbot
Core principles:

Tool fidelity (strict use of Spec-Kit Plus, Claude Code, Docusaurus)
Seamless integration (RAG chatbot fully embedded and functional)
Accuracy (chatbot responses grounded in book content only)
Deployability (live on GitHub Pages)

Key standards:

Framework: Docusaurus + GitHub Pages deployment
Backend: FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier
Chatbot SDK: OpenAI Agents/ChatKit
Features: Answer queries on full book or user-selected text
Code: Version-controlled on GitHub

Constraints:

No external data sources for RAG
Exclusive use of specified tools/services
Minimum 10 functional test queries

Success criteria:

Book live and accessible on GitHub Pages
Embedded RAG chatbot operational
Accurate responses to full-content and selected-text queries
Zero critical deployment errors
All components traceable to project requirements
```

## Response snapshot

Created the initial project constitution file (.specify/memory/constitution.md) with the specified core principles, standards, constraints, and success criteria. The constitution includes six core principles: Tool Fidelity, Seamless Integration, Accuracy, Deployability, Code Quality, and Performance Efficiency. It also defines additional standards, development workflow, and governance rules.

## Outcome

- ✅ Impact: Created initial project constitution file with all specified principles and guidelines
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md created
- 🔁 Next prompts: Proceed with project setup based on constitution
- 🧠 Reflection: Constitution properly captures all specified requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin project specification based on constitution