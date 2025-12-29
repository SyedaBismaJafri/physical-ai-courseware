---
id: 2
title: Clarify-Implementation-Constraints
stage: clarify
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: physical-ai-courseware
branch: master
user: bisma
command: /sp.clearify
labels: ["constraints", "implementation", "guidelines", "directory-structure", "versioning"]
links:
  spec: specs/physical-ai-courseware/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/physical-ai-courseware/implementation-guidelines.md
tests:
 - specs/physical-ai-courseware/implementation-guidelines.md
---

## Prompt

/sp.clearify Goal: Remove technical friction and folder confusion.

"Clarify these constraints for the implementation:

Directory Isolation: Ensure npm install and Docusaurus commands are only run inside /frontend.

Pathing: When the Docusaurus content references code snippets, use relative paths pointing to the /backend folder.

Version Sync: Confirm the use of ROS 2 Humble and Gazebo Harmonic/Garden to ensure compatibility with Isaac Sim 2023+.

Authentication: Assume API keys for LLMs (OpenAI/Claude) will be handled via .env files in /backend."

## Response snapshot

Created comprehensive implementation guidelines document that clarifies:

1. Directory isolation requirements specifying that npm and Docusaurus commands must only run inside /frontend
2. Pathing conventions for cross-referencing between frontend and backend directories
3. Version compatibility constraints requiring ROS 2 Humble and Gazebo Garden/Harmonic for Isaac Sim 2023+ compatibility
4. Authentication approach using .env files in /backend for LLM API keys
5. Best practices and troubleshooting guidance

The implementation guidelines document provides clear, actionable constraints that remove technical friction and folder confusion for developers.

## Outcome

- ✅ Impact: Created clear implementation constraints that address all specified requirements
- 🧪 Tests: Not applicable for documentation work
- 📁 Files: 1 comprehensive guidelines document created
- 🔁 Next prompts: Could create specific implementation guides for each module
- 🧠 Reflection: The guidelines document addresses all four constraint areas effectively

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin module-specific implementation guides