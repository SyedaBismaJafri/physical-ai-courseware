# Physical AI Courseware

This repository contains the courseware for a comprehensive Physical AI curriculum covering robotics, simulation, and AI systems.

## Project Structure

```
physical-ai-courseware/
├── frontend/           # Docusaurus documentation site
├── backend/            # Backend services and integration components
└── .github/workflows/  # GitHub Actions for deployment
```

### Frontend

The frontend directory contains a Docusaurus-based documentation site with the following modules:

1. **Physical AI Overview** - Introduction to Physical AI concepts
2. **Module 1: ROS 2** - Robot Operating System fundamentals
3. **Module 2: Simulation** - Simulation environments and tools
4. **Module 3: NVIDIA Isaac** - NVIDIA Isaac ecosystem
5. **Module 4: VLA Brain** - Vision-Language-Action models
6. **Capstone** - Final integrated project

### Backend

The backend directory contains service components:

- `ros2_ws/` - ROS 2 workspace with sample packages
- `isaac_sim_scripts/` - NVIDIA Isaac simulation scripts and configurations
- `vla_service/` - Vision-Language-Action service with LLM and Whisper integration

## Getting Started

### Frontend Development

1. Navigate to the frontend directory: `cd frontend`
2. Install dependencies: `npm install`
3. Start the development server: `npm start`

### Backend Development

The backend services are organized in the `backend/` directory with the following structure:

- `ros2_ws/` - ROS 2 workspace (follow ROS 2 installation guide)
- `isaac_sim_scripts/` - NVIDIA Isaac simulation scripts
- `vla_service/` - Python service with LLM integration

## Deployment

The frontend is automatically deployed to GitHub Pages via GitHub Actions when changes are pushed to the main branch.
