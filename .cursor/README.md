# Cursor IDE Configuration

Custom configuration for Cursor IDE optimized for ROS 2 development.

---

## 📁 Structure

```
.cursor/
├── README.md           # This file
├── context.md          # Dev container context
├── mcp.json            # MCP server configuration
├── commands/           # Custom commands
│   ├── explore.md
│   ├── plan.md
│   ├── code.md
│   ├── test.md
│   ├── iterate.md
│   ├── validate-*.md
│   ├── status.md
│   └── help.md
├── rules/              # Development rules
│   ├── cpp.mdc
│   ├── python.mdc
│   ├── cuda.mdc
│   ├── ros2-generic.mdc
│   ├── ros2-package-structure.mdc
│   ├── ros2-diagnostics.mdc
│   ├── documentation.mdc
│   ├── software-testing.mdc
│   └── workflows.mdc
├── blocklist           # Excluded patterns
├── ide_state.json      # IDE state tracking
└── unified_repo_list.json  # Repository list
```

---

## 🎯 Purpose

This configuration provides:

1. **Context awareness**: Dev container setup and MCP servers
2. **Custom commands**: Workflow automation (explore, plan, code, test, iterate, validate)
3. **Development rules**: C++, Python, CUDA, ROS 2, documentation standards
4. **MCP integration**: Multiple servers for enhanced capabilities

---

## 🔧 Custom Commands

### Available Commands

| Command | Purpose |
|---------|---------|
| `/explore [target]` | Analyze requirements and codebase |
| `/plan [package]` | Create implementation plan |
| `/code [package] [phase]` | Implement solution |
| `/test [package] [type]` | Run tests |
| `/iterate [package] [focus]` | Refine and optimize |
| `/validate-architecture` | Check Clean Architecture compliance |
| `/validate-package` | Complete package validation |
| `/validate-tests` | Test coverage validation |
| `/validate-workflow` | Workflow compliance check |
| `/status` | Display project status |
| `/help` | Show command help |

### Command Workflow

```
/explore → /plan → /code → /test → /iterate → /validate → /status
```

---

## 📜 Development Rules

### Always Applied Rules

All rules in `.cursor/rules/` are configured with `alwaysApply: true`:

- **cpp.mdc**: Modern C++20 standards, Clean Architecture
- **python.mdc**: PEP 8, type hints, Flask/ML patterns
- **cuda.mdc**: GPU programming standards
- **ros2-generic.mdc**: Generic ROS 2 patterns
- **ros2-package-structure.mdc**: Package organization
- **ros2-diagnostics.mdc**: Diagnostics and monitoring
- **documentation.mdc**: Documentation standards (English, concise)
- **software-testing.mdc**: ISTQB-based testing principles
- **workflows.mdc**: Core workflows (docs, simplification, readiness)

### Key Principles

1. **Clean Architecture**: Business logic separate from frameworks
2. **No ROS in Core**: Domain layer has NO ROS2 dependencies
3. **Concise Docs**: Max 2 pages per document, English only
4. **Test Coverage**: >80% for business logic, E2E for system
5. **Type Safety**: C++ concepts, Python type hints

---

## 🌐 MCP Servers

### Configured Servers

1. **ros-mcp-server**
   - Purpose: ROS 2 system introspection
   - Command: `uv run python ros-general.py`
   - Location: `/workspace/.devcontainer/ros-mcp-server/`
   - Built: During devcontainer creation

2. **puppeteer**
   - Purpose: Web automation
   - Command: `npx -y @modelcontextprotocol/server-puppeteer`

3. **filesystem**
   - Purpose: File operations
   - Command: `npx -y @modelcontextprotocol/server-filesystem`
   - Paths: ros_ws, architecture, iotbridge

4. **sequential-thinking**
   - Purpose: Enhanced reasoning
   - Command: `npx -y @modelcontextprotocol/server-sequential-thinking`

5. **memory**
   - Purpose: Persistent memory
   - Command: `npx -y @modelcontextprotocol/server-memory`

6. **chrome-mcp**
   - Purpose: Chrome control
   - Type: streamableHttp
   - URL: http://127.0.0.1:12306/mcp

### Usage

MCP servers provide enhanced capabilities:
- ROS 2 system introspection (topics, nodes, services, parameters)
- File operations across multiple workspaces
- Web automation for testing
- Enhanced reasoning and memory

---

## 🎨 IDE State

### Tracked Information

**File**: `ide_state.json`

- Recently viewed files
- Active workspace context
- Navigation history

### Recently Viewed (Example)

- CUDA rules
- MCP configuration
- Command definitions
- Workflow standards
- Motor control configs
- IoT Hub implementations

---

## 📝 Usage Guidelines

### When Starting Development

1. Check context: Review `context.md` for environment info
2. Understand workflow: Use custom commands for structured development
3. Follow rules: Standards are automatically applied
4. Use MCPs: Leverage servers for enhanced capabilities

### Command Usage Examples

```bash
# Start new feature
/explore "motor control improvements"
/plan motor_control
/code motor_control core
/code motor_control ros2
/test motor_control all
/validate-package motor_control

# Check status
/status

# Iterate on performance
/iterate motor_control performance
```

### Best Practices

1. **Use commands**: Structured workflow ensures quality
2. **Follow rules**: Clean Architecture, testing, documentation
3. **English only**: All documentation in English
4. **Concise docs**: Keep documentation focused and actionable
5. **Test coverage**: Aim for >80% business logic coverage

---

## 🔄 Updating Configuration

### Adding Rules

1. Create `.mdc` file in `rules/`
2. Set `alwaysApply: true` in frontmatter
3. Follow existing rule format
4. Keep code-free (principles only)

### Adding Commands

1. Create `.md` file in `commands/`
2. Follow command template structure
3. Document usage and examples
4. Update this README

### Modifying MCP Servers

1. Edit `.cursor/mcp.json`
2. Test server connection
3. Document any new capabilities
4. Update `context.md` if needed

---

## 🆘 Troubleshooting

### Commands not working

- Check command syntax in `commands/` directory
- Ensure file format is correct
- Reload Cursor IDE

### Rules not applied

- Verify `alwaysApply: true` in frontmatter
- Check glob patterns match files
- Reload IDE window

### MCP servers not available

- Check server installation (npx packages)
- Verify paths in mcp.json
- Check server logs

---

## 📚 Related Documentation

- **Dev Container**: `../docs/setup/README.md`
- **Workspace Info**: `context.md`
- **MCP Config**: `mcp.json`
- **Full Docs**: `../docs/README.md`

---

**Configuration Version**: 2.0.0
**Last Updated**: 2025-10-23
**Compatibility**: Cursor IDE with MCP support
