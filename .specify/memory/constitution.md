<!-- SYNC IMPACT REPORT:
Version change: 1.0.0 → 1.0.0 (initial creation)
Added sections: Core Principles (6), Additional Standards, Development Workflow, Governance
Templates requiring updates: N/A (initial creation)
Follow-up TODOs: None
-->
# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Tool Fidelity
Strict adherence to Spec-Kit Plus, Claude Code, and Docusaurus tools only; No deviation from specified technology stack without formal amendment; All development activities must utilize designated tools and processes.

### Seamless Integration
RAG chatbot must be fully embedded and functionally integrated within the book interface; Zero friction user experience between book content and chatbot interaction; All integration points must be tested and validated.

### Accuracy
Chatbot responses MUST be grounded in book content only; No external data sources allowed for RAG functionality; Responses must cite specific book sections when providing information.

### Deployability
Solution must be deployable on GitHub Pages with zero configuration required; All backend services must be operational post-deployment; Continuous deployment pipeline must be established and maintained.

### Code Quality
All code must be version-controlled on GitHub with proper branching strategy; Code reviews required for all changes; Automated testing coverage must meet minimum standards before merge.

### Performance Efficiency
Backend services must respond within acceptable latency thresholds; Resource consumption must remain within free tier limitations; Scalability considerations must be addressed in design.

## Additional Standards

**Framework**: Docusaurus + GitHub Pages deployment
**Backend**: FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier
**Chatbot SDK**: OpenAI Agents/ChatKit
**Features**: Answer queries on full book or user-selected text
**Code**: Version-controlled on GitHub

**Constraints**:
- No external data sources for RAG
- Exclusive use of specified tools/services
- Minimum 10 functional test queries required

## Development Workflow

**Specification**: All features must be documented in spec files before implementation
**Planning**: Architecture decisions must be recorded in plan files
**Tasks**: Implementation must follow task breakdown with acceptance criteria
**Testing**: Functional tests must validate all core features before deployment
**Documentation**: All user-facing features must have corresponding documentation

## Governance

This constitution governs all project decisions and supersedes any conflicting practices. All team members must comply with these principles. Amendments require formal documentation, approval process, and migration plan when applicable. All pull requests and code reviews must verify constitutional compliance.

All development activities must trace back to project requirements. Code quality standards must be maintained throughout the development lifecycle.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16