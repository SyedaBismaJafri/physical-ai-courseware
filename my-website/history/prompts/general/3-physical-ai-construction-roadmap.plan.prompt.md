---
id: 3
title: Physical-AI-Construction-Roadmap
stage: plan
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: physical-ai-courseware
branch: master
user: bisma
command: /sp.plan
labels: ["planning", "roadmap", "construction", "docusaurus", "backend-setup"]
links:
  spec: specs/physical-ai-courseware/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/physical-ai-courseware/plan.md
tests:
 - specs/physical-ai-courseware/plan.md
---

## Prompt

/sp.plan Goal: Create the step-by-step construction roadmap.

"Plan the build sequence:

Init: Create /frontend using npx create-docusaurus@latest frontend classic.

Backend Setup: Create /backend with subfolders for nodes, config, and models.

Content Mapping: Map each book Module to a specific folder in frontend/docs.

Infrastructure: Write the deploy.yml in .github/workflows/ that specifically targets the frontend folder for building.

Validation: Perform a dry-run build of the frontend to ensure no broken links to backend scripts."

## Response snapshot

Created comprehensive construction roadmap document that outlines the step-by-step build sequence for the Physical AI Courseware project:

1. Phase 1: Initialization - Creating the frontend Docusaurus site using the classic template
2. Phase 2: Backend Setup - Creating the backend directory with subfolders for nodes, config, and models
3. Phase 3: Content Mapping - Mapping each book module to specific folders in frontend/docs
4. Phase 4: Infrastructure - Writing the GitHub Actions deploy.yml that targets the frontend folder
5. Phase 5: Validation - Performing dry-run builds to ensure no broken links

The roadmap includes implementation timeline, success criteria, risk mitigation strategies, and next steps for execution.

## Outcome

- ✅ Impact: Created detailed construction roadmap addressing all requested build sequence phases
- 🧪 Tests: Not applicable for planning document
- 📁 Files: 1 comprehensive roadmap document created
- 🔁 Next prompts: Could begin implementation of the roadmap phases
- 🧠 Reflection: The roadmap provides clear, actionable steps for building the project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin executing Phase 1 of the roadmap