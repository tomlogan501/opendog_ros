# Dev Container Context —

## 🎯 Project Overview

**Project**: ROS 2 Humble Development Container
**Purpose**: Standardized development environment for ROS 2 robotics projects
**Type**: Development tools and workspace infrastructure

---

## 🐋 Dev Container Setup

### Base Environment

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS**: ROS 2 Humble Hawksbill
- **Shell**: Bash with custom aliases
- **User**: developer (non-root, UID 1000)

### Key Features

- **Auto-configured**: Post-create scripts handle setup
- **Persistent**: Volumes for bash history, Gazebo config
- **SSH Forwarding**: Host SSH keys available (read-only)
- **Port Forwarding**: 8080, 5173, 3000, 8765

---

## 📦 Workspace Structure

```
/workspace/
├── .devcontainer/       # Container configuration
├── .vscode/             # VS Code settings & tasks
├── .cursor/             # Cursor IDE configuration
├── ros_ws/              # ROS 2 workspace
│   └── src/             # Robot packages
├── scripts/             # Utility scripts
└── docs/                # Documentation
```

---

## 🛠️ Available Tools

### Build System
- **colcon**: ROS 2 build tool
- **CMake**: Build configuration
- **rosdep**: Dependency management

### ROS 2 Packages
- ros2_control, ros2_controllers
- rqt, rqt-common-plugins
- foxglove_bridge
- libphidget22

### Development Tools
- clangd, clang-format
- Python tools (pytest, autopep8)
- Git, SSH

---

## 🎨 VS Code Integration

### Extensions (24 total)

**C++ Development**: cpptools, clangd, clang-format, cmake-tools
**Python**: python, debugpy, autopep8, pyright
**ROS**: vscode-ros
**Git**: git-graph, gitblame, gitlens
**Docker**: docker, remote-containers
**Testing**: catch2-test-adapter

### Tasks (16+ available)

- Build & Clean
- Test execution
- ROS node management
- Foxglove visualization
- Launch configurations

### Bash Aliases

```bash
ws        # cd /workspace/ros_ws
wsb       # colcon build --symlink-install
wsc       # clean workspace
wst       # test workspace
wsi       # workspace info
roskill   # kill all ROS nodes
rosmon    # monitor ROS system
rossrc    # source workspace
```

---

## 🌐 MCP Servers

### Available MCPs

1. **ros-mcp-server**: ROS 2 introspection via rosbridge
2. **puppeteer**: Web automation and testing
3. **filesystem**: File operations on host workspaces
4. **sequential-thinking**: Enhanced reasoning
5. **memory**: Persistent memory across sessions
6. **github**: GitHub integration
7. **chrome-mcp**: Chrome browser control
8. **excalidraw**: Diagram creation

### Configuration

**Location**: `.cursor/mcp.json`


---

## 🔄 Development Workflow

### Standard Cycle

```bash
# 1. Start
roskill              # Clean old nodes
wsi                  # Check status

# 2. Develop
# Edit code...
wsb --packages-select my_package

# 3. Test
wst --package my_package

# 4. Debug
rosmon --watch       # Monitor
ros2 launch my_package my_launch.py

# 5. Visualize
bash scripts/foxglove_start.sh
# Open https://studio.foxglove.dev
# Connect to ws://localhost:8765

# 6. Commit
git add .
git commit -m "feat: description"
git push origin branch
```

---

## 📝 Documentation Structure

### Organization

```
docs/
├── README.md           # Index
├── setup/              # Getting started
├── usage/              # Daily workflows
├── tools/              # Tool guides
└── reference/          # Technical reference
```

### Standards

- **Language**: English only
- **Format**: Markdown
- **Length**: Max 2 pages per document
- **Style**: Concise, actionable

---

## 🎯 Common Tasks

### Build & Test
```bash
wsb                     # Build all
wsb --packages-select PKG  # Build specific
wst                     # Test all
wst --package PKG       # Test specific
```

### ROS Management
```bash
roskill                 # Kill all nodes
rosmon --watch          # Monitor continuously
ros2 node list          # List active nodes
ros2 topic list         # List active topics
```

### Package Operations
```bash
bash scripts/scale_workspace.sh list      # List packages
bash scripts/scale_workspace.sh analyze   # Analyze deps
```

### Visualization
```bash
bash scripts/foxglove_start.sh   # Start bridge
bash scripts/foxglove_stop.sh    # Stop bridge
```

---

## 🔐 Git Configuration

### SSH (Recommended)
- Host keys mounted from WSL
- SSH agent forwarding configured
- Automatic in postCreateCommand

### HTTPS (Alternative)
```bash
bash scripts/git_push_https.sh
# Create GitLab personal access token
# Use token as password
```

---

## 🐛 Troubleshooting

### Quick Fixes

**Build fails**: `wsc --all && wsb`
**Tests fail**: `wst --verbose`
**Nodes stuck**: `roskill`
**Bridge issues**: `pkill -f foxglove_bridge && bash scripts/foxglove_start.sh`
**Container slow**: `docker system prune -a`

### Full Recovery

```bash
roskill
cd /workspace/ros_ws
rm -rf build/ install/ log/
bash .devcontainer/utils/install_dependencies.sh
rosdep install --from-paths src --ignore-src -y
wsb
```

---

## 📊 Project Packages

### Control Packages
- **motor_control**: Motor control (3 motors)
- **pump_control**: Pump control with flow correction
- **turret**: Turret coordination system

### Interface Packages
- **motor_control_interfaces**: Motor messages/services
- **pump_control_interfaces**: Pump messages/services
- **turret_interfaces**: Turret messages/services

### Architecture
- **Pattern**: Clean Architecture (core/ + ros2_interface/)
- **Business Logic**: NO ROS2 dependencies
- **Testing**: Unit tests (core) + E2E tests (ROS2)

---

## 🔑 Key Files

### Container
- `.devcontainer/devcontainer.json` - Main config
- `.devcontainer/Dockerfile` - Image definition
- `.devcontainer/utils/postCreateCommand.sh` - Setup script

### VS Code
- `.vscode/tasks.json` - Task definitions
- `.vscode/settings.json` - Editor settings
- `.vscode/extensions.json` - Extension list

### Workspace
- `workspace.code-workspace` - Multi-root config
- `.gitignore` - Ignored files

---

## 🎓 Best Practices

### Development
- Follow Clean Architecture principles
- Separate business logic from ROS2
- Use bash aliases for speed
- Test incrementally

### Documentation
- Keep docs concise (max 2 pages)
- Use English only
- Update docs with code changes
- Link related documents

### Git
- Use feature branches
- Write clear commit messages
- Test before pushing
- Use SSH when possible

---

**Version**: 2.0.0
**Last Updated**: 2025-10-23
**Maintainer**:  Team
