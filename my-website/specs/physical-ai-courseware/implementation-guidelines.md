# Physical AI Courseware - Implementation Guidelines

## 1. Overview

This document clarifies the implementation constraints to remove technical friction and folder confusion for the Physical AI Courseware project.

## 2. Directory Isolation Requirements

### 2.1 Frontend Directory Isolation
- **npm commands must only be run inside `/frontend` directory**
  - `npm install` → Execute only in `/frontend` directory
  - `npm start` → Execute only in `/frontend` directory
  - `npm run build` → Execute only in `/frontend` directory
  - `npm run deploy` → Execute only in `/frontend` directory

- **Docusaurus commands must only be run inside `/frontend` directory**
  - `npx docusaurus start` → Execute only in `/frontend` directory
  - `npx docusaurus build` → Execute only in `/frontend` directory
  - `npx docusaurus deploy` → Execute only in `/frontend` directory
  - `npx docusaurus swizzle` → Execute only in `/frontend` directory

### 2.2 Backend Directory Isolation
- **Python commands must only be run inside `/backend` or its subdirectories**
  - `pip install` → Execute in `/backend/vla_service` directory
  - `python` scripts → Execute in appropriate backend subdirectories
  - ROS 2 commands → Execute in `/backend/ros2_ws` directory

### 2.3 Root Directory Commands
- **Git operations** → Execute in root directory only
- **Workspace management** → Execute in root directory only
- **Documentation** → Execute in root directory only

## 3. Pathing Conventions

### 3.1 Cross-Directory References from Frontend to Backend
When Docusaurus content references code snippets from the `/backend` folder, use the following relative paths:

```
# Example for referencing ROS 2 code snippets in documentation
[ROS 2 Example](/backend/ros2_ws/src/example_package/example_node.py)

# Example for referencing Isaac Sim scripts
[Isaac Sim Script](/backend/isaac_sim_scripts/launch/example_launch.py)

# Example for referencing VLA service code
[VLA Service API](/backend/vla_service/api/example_endpoint.py)
```

### 3.2 Docusaurus Static Assets
- Place static assets (code snippets, diagrams, images) in `/frontend/static/`
- Reference from Docusaurus as: `/img/filename` or `/docs/filename`
- For backend code examples, copy relevant snippets to `/frontend/static/examples/`

### 3.3 Documentation Linking
- Internal Docusaurus links: `../module1-ros2/page` or `/docs/module1-ros2/page`
- Backend file references: Use GitHub repository links or static copies
- Avoid direct filesystem references to `/backend` from Docusaurus

## 4. Version Compatibility Constraints

### 4.1 ROS 2 Version Requirement
- **ROS 2 Humble Hawksbill** (LTS version) - Required
  - Release: May 2022
  - Support: Until May 2027
  - Ubuntu 22.04 LTS compatibility
  - Full compatibility with Isaac Sim 2023+

### 4.2 Gazebo Version Requirements
- **Gazebo Garden** OR **Gazebo Harmonic** - Required
  - Gazebo Garden: Supports Isaac Sim 2023.1+
  - Gazebo Harmonic: Supports Isaac Sim 2023.2+
  - Both provide proper ROS 2 Humble integration
  - Both offer compatible physics engines and sensors

### 4.3 Isaac Sim Compatibility
- **Isaac Sim 2023.1+** - Required
  - Must be compatible with ROS 2 Humble
  - Should support Gazebo Garden/Harmonic
  - NVIDIA RTX GPU recommended for rendering
  - CUDA 11.8+ required

### 4.4 Python Version Requirements
- **Python 3.10 or 3.11** - Required for ROS 2 Humble compatibility
- Avoid Python 3.12+ until full ROS 2 Humble support is confirmed

### 4.5 Node.js Version Requirements
- **Node.js 18+** - Required for Docusaurus v3 compatibility
- Recommended: Node.js 18.x LTS or 20.x LTS

## 5. Authentication Approach

### 5.1 API Key Management
- **Location**: `.env` files in `/backend/vla_service/` directory only
- **Format**: Standard environment variable format
```
OPENAI_API_KEY=your_openai_api_key_here
ANTHROPIC_API_KEY=your_anthropic_api_key_here
WHISPER_API_KEY=your_whisper_api_key_here
```

### 5.2 Environment File Security
- **Never commit** `.env` files to version control
- Add `/backend/vla_service/.env` to `.gitignore`
- Create `.env.example` with template for developers
- Use Docker secrets for production deployments

### 5.3 API Key Access in Code
- **Backend services only**: API keys should only be accessed in `/backend/vla_service/`
- **Frontend isolation**: Frontend should never have direct access to API keys
- **Service-to-service communication**: Use internal API endpoints, not direct LLM calls from frontend

### 5.4 Example .env File Structure
```
# OpenAI Configuration
OPENAI_API_KEY=
OPENAI_MODEL=gpt-4-turbo

# Anthropic Configuration
ANTHROPIC_API_KEY=
ANTHROPIC_MODEL=claude-3-opus-20240229

# Whisper Configuration
WHISPER_API_KEY=
WHISPER_MODEL=whisper-1

# Database Configuration (if needed)
DATABASE_URL=

# Other Service Configuration
JWT_SECRET=
```

## 6. Implementation Best Practices

### 6.1 Development Workflow
1. Always navigate to the correct directory before running commands
2. Use absolute paths when referencing files across directories
3. Test backend services independently before frontend integration
4. Validate version compatibility before starting development

### 6.2 Testing Approach
- Test frontend and backend separately
- Use mock services for cross-service testing
- Validate path references in documentation
- Verify environment configuration before deployment

### 6.3 Troubleshooting Common Issues
- **Command not found**: Ensure you're in the correct directory
- **Module import errors**: Check Python version and ROS 2 setup
- **Path resolution issues**: Verify relative path conventions
- **API key errors**: Confirm .env file placement and format

## 7. Quick Reference Commands

### Frontend Development
```bash
cd frontend
npm install
npm start
```

### Backend Development
```bash
cd backend/vla_service
pip install -r requirements.txt
python -m api.main
```

### ROS 2 Setup
```bash
cd backend/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Isaac Sim Setup
```bash
# Follow NVIDIA Isaac Sim installation guide
# Ensure Gazebo Garden/Harmonic compatibility
```