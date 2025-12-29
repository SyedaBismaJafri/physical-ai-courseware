---
id: 1
title: Define-Physical-AI-Workspace-Structure
stage: spec
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: physical-ai-courseware
branch: master
user: bisma
command: /sp.specify
labels: ["workspace", "structure", "specification", "docusaurus", "physical-ai"]
links:
  spec: history/adr/physical-ai-workspace-structure.adr
  ticket: null
  adr: null
  pr: null
files:
 - specs/physical-ai-courseware/spec.md
 - .github/workflows/deploy.yml
 - frontend/package.json
 - README.md
 - docusaurus.config.js
 - sidebars.js
tests:
 - specs/physical-ai-courseware/spec.md
---

## Prompt

/sp.specify Goal: Define the exact requirements for both directories.

"Specify requirements for the workspace:

Root Directory: Contains /frontend and /backend.

Frontend Folder: A Docusaurus site. Sidebar must have categories: 'Physical AI Overview', 'Module 1: ROS 2', 'Module 2: Simulation', 'Module 3: NVIDIA Isaac', 'Module 4: VLA Brain', and 'Capstone'.

Backend Folder: Must contain subdirectories for /ros2_ws (workspace), /isaac_sim_scripts, and /vla_service (LLM/Whisper logic).

Deployment: Configure GitHub Actions to build from the frontend/ directory and deploy to GitHub Pages."

## Response snapshot

Created comprehensive requirements specification for the Physical AI Courseware workspace, including:

1. Detailed directory structure requirements for root, frontend, and backend
2. Specific Docusaurus sidebar categories as requested
3. Backend subdirectory structure with ros2_ws, isaac_sim_scripts, and vla_service
4. GitHub Actions workflow for deployment to GitHub Pages
5. Updated Docusaurus configuration to match the courseware theme
6. Updated sidebar to include the 6 required categories
7. Updated README with project structure documentation

The specification document (specs/physical-ai-courseware/spec.md) includes detailed requirements for both frontend and backend components, deployment configuration, acceptance criteria, and constraints.

## Outcome

- ✅ Impact: Created comprehensive workspace specification with all required components
- 🧪 Tests: Not applicable for specification work
- 📁 Files: 7 files created/modified to implement the workspace structure
- 🔁 Next prompts: Could implement the actual modules or backend services
- 🧠 Reflection: The specification clearly defines all requirements from the original request

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin implementation of first module content