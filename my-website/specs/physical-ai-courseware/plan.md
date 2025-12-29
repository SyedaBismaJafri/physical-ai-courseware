# Physical AI Courseware - Construction Roadmap

## 1. Overview

This document provides a step-by-step construction roadmap for building the Physical AI Courseware project, including initialization, setup, content mapping, infrastructure, and validation.

## 2. Build Sequence Plan

### Phase 1: Initialization
**Goal**: Create the frontend Docusaurus site using the classic template

**Steps**:
1. Navigate to the project root directory
2. Run the Docusaurus initialization command (if not already done):
   ```bash
   npx create-docusaurus@latest frontend classic
   ```
3. Verify the frontend directory structure:
   - `frontend/package.json` - Contains Docusaurus dependencies
   - `frontend/docusaurus.config.js` - Main configuration file
   - `frontend/sidebars.js` - Sidebar navigation configuration
   - `frontend/src/` - Custom source files
   - `frontend/static/` - Static assets
   - `frontend/docs/` - Documentation content

**Expected Outcome**: A functional Docusaurus site in the frontend directory ready for content development.

### Phase 2: Backend Setup
**Goal**: Create the backend directory with required subfolders for nodes, config, and models

**Steps**:
1. Create the backend directory structure:
   ```bash
   mkdir -p backend/ros2_ws/src
   mkdir -p backend/isaac_sim_scripts/{nodes,config,models}
   mkdir -p backend/vla_service/{nodes,config,models}
   ```
2. Set up ROS 2 workspace:
   - Initialize the ROS 2 workspace in `backend/ros2_ws/`
   - Create sample ROS 2 packages in `backend/ros2_ws/src/`
3. Set up Isaac Sim scripts:
   - Create launch files in `backend/isaac_sim_scripts/nodes/`
   - Create configuration files in `backend/isaac_sim_scripts/config/`
   - Create model files in `backend/isaac_sim_scripts/models/`
4. Set up VLA service:
   - Create API endpoints in `backend/vla_service/nodes/`
   - Create configuration files in `backend/vla_service/config/`
   - Create model loading utilities in `backend/vla_service/models/`

**Expected Outcome**: A well-organized backend directory with proper subfolder structure for nodes, configuration, and models.

### Phase 3: Content Mapping
**Goal**: Map each book Module to a specific folder in frontend/docs

**Steps**:
1. Create module-specific directories in `frontend/docs/`:
   - `frontend/docs/overview/` - Physical AI Overview module
   - `frontend/docs/module1-ros2/` - Module 1: ROS 2
   - `frontend/docs/module2-simulation/` - Module 2: Simulation
   - `frontend/docs/module3-isaac/` - Module 3: NVIDIA Isaac
   - `frontend/docs/module4-vla/` - Module 4: VLA Brain
   - `frontend/docs/capstone/` - Capstone project
2. Map content to appropriate directories:
   - Overview content → `frontend/docs/overview/`
   - ROS 2 concepts and tutorials → `frontend/docs/module1-ros2/`
   - Simulation exercises → `frontend/docs/module2-simulation/`
   - Isaac tutorials → `frontend/docs/module3-isaac/`
   - VLA examples → `frontend/docs/module4-vla/`
   - Final project → `frontend/docs/capstone/`
3. Update sidebar configuration to reflect the module structure
4. Create navigation links between modules for sequential learning

**Expected Outcome**: Organized documentation structure that maps directly to the book's module organization.

### Phase 4: Infrastructure Setup
**Goal**: Write the deploy.yml in `.github/workflows/` that specifically targets the frontend folder for building

**Steps**:
1. Create the GitHub Actions workflow directory:
   ```bash
   mkdir -p .github/workflows
   ```
2. Create the deployment workflow file `.github/workflows/deploy.yml`:
   ```yaml
   name: Deploy to GitHub Pages

   on:
     push:
       branches: [main]
     pull_request:
       branches: [main]

   jobs:
     deploy:
       name: Deploy to GitHub Pages
       runs-on: ubuntu-latest
       defaults:
         run:
           working-directory: ./frontend
       steps:
         - uses: actions/checkout@v4
         - uses: actions/setup-node@v4
           with:
             node-version: 18
             cache: npm
             cache-dependency-path: frontend/package-lock.json

         - name: Install dependencies
           run: npm ci
         - name: Build website
           run: npm run build

         - name: Upload Build Artifact
           uses: actions/upload-pages-artifact@v3
           with:
             path: frontend/build

         - name: Deploy to GitHub Pages
           id: deployment
           uses: actions/deploy-pages@v4
   ```
3. Ensure the workflow specifically targets the frontend directory for all operations
4. Test the workflow configuration for proper permissions and access

**Expected Outcome**: Automated deployment pipeline that builds and deploys the frontend Docusaurus site to GitHub Pages.

### Phase 5: Validation
**Goal**: Perform a dry-run build of the frontend to ensure no broken links to backend scripts

**Steps**:
1. Run a local build of the frontend:
   ```bash
   cd frontend
   npm install
   npm run build
   ```
2. Check for build errors and warnings
3. Verify all internal links in the documentation:
   - Run Docusaurus link checker if available
   - Manually verify navigation between pages
4. Validate external links to backend resources:
   - Check that all references to backend scripts are properly documented
   - Ensure any backend code examples are properly formatted and accessible
5. Run a local server to test navigation:
   ```bash
   npm start
   ```
6. Perform cross-browser testing to ensure compatibility
7. Verify that all sidebar navigation works correctly
8. Check that search functionality works across all modules

**Expected Outcome**: A fully functional, error-free frontend build with no broken links or navigation issues.

## 3. Implementation Timeline

### Week 1: Initialization and Backend Setup
- Complete Phase 1: Initialization
- Complete Phase 2: Backend Setup
- Set up development environment

### Week 2: Content Mapping and Infrastructure
- Complete Phase 3: Content Mapping
- Complete Phase 4: Infrastructure Setup
- Begin initial content creation

### Week 3: Validation and Testing
- Complete Phase 5: Validation
- Perform comprehensive testing
- Address any issues found during validation

## 4. Success Criteria

### Technical Requirements
- [ ] Frontend Docusaurus site builds without errors
- [ ] Backend directory structure is properly organized
- [ ] Content is properly mapped to module directories
- [ ] GitHub Actions workflow deploys successfully
- [ ] No broken links or navigation issues
- [ ] All modules are accessible through sidebar navigation

### Quality Requirements
- [ ] Documentation is well-structured and easy to navigate
- [ ] Code examples are properly formatted and functional
- [ ] All external dependencies are properly documented
- [ ] Security best practices are followed for API keys
- [ ] Performance benchmarks are met for site loading

### Deployment Requirements
- [ ] GitHub Pages deployment is automated
- [ ] Site is accessible to target audience
- [ ] Documentation is properly versioned
- [ ] Error handling is implemented appropriately

## 5. Risk Mitigation

### Potential Risks
1. **Version Compatibility Issues**: Ensure all tools use compatible versions (ROS 2 Humble, Gazebo Garden/Harmonic, Isaac Sim 2023+)
2. **Dependency Conflicts**: Use isolated environments for frontend and backend
3. **Link Rot**: Maintain static copies of important backend code examples
4. **Deployment Failures**: Test workflow locally before enabling automatic deployment

### Mitigation Strategies
1. Document all version requirements clearly in implementation guidelines
2. Use virtual environments and containerization where appropriate
3. Create backup copies of critical code examples in the frontend static folder
4. Test deployment workflow in a separate branch before merging to main

## 6. Next Steps

1. Execute Phase 1: Initialization to create the frontend
2. Execute Phase 2: Backend Setup to organize the backend structure
3. Execute Phase 3: Content Mapping to organize documentation
4. Execute Phase 4: Infrastructure Setup to automate deployment
5. Execute Phase 5: Validation to ensure quality and functionality
6. Begin content creation for each module following the established structure