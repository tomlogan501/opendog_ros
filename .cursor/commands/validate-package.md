# Validate Package Command —

## 🎯 Purpose

Complete validation of ROS2 package (structure, code, tests, documentation).

## 🚀 Usage

```bash
/validate-package [package_name]
```

## 📋 What it validates

### 1. Package Structure

- Correct directory organization (core/ + ros2_interface/)
- main.cpp contains only initialization
- Business logic has NO rclcpp dependencies
- Launch files in launch/ directory
- Config files in config/ directory
- Tests in test/ directory

### 2. Documentation

- README.md exists and complete (max 2 pages)
- SYSTEM_STATUS.md updated
- architecture.md complete (max 2 pages)
- system-architecture.md with valid Mermaid diagrams
- messages.md documents all interfaces
- e2e-testing-strategy.md with test scenarios
- All page limits respected
- No placeholder text remaining

### 3. Build System

- CMakeLists.txt follows template
- Separate libraries for core and ros2
- Core library has NO rclcpp dependency
- package.xml complete and valid
- All dependencies declared

### 4. Compilation

- `colcon build --packages-select package_name` succeeds
- No compilation warnings
- All targets build successfully

### 5. Tests

- Unit tests exist for business logic
- Unit tests have NO rclcpp dependencies
- E2E tests use launch_testing
- `colcon test --packages-select package_name` succeeds
- All tests pass
- Test coverage > 80% for core/

### 6. Code Quality

- Business logic separated from ROS2
- No ROS2 types in core/ layer
- Proper error handling
- Follow C++ or Python standards
- Copyright headers present

## 📤 Output

- Validation report with passed/failed checks
- Quality metrics and recommendations
- Next steps for improvement

## 🔍 Examples

```bash
/validate-package pump_control
/validate-package turret_control
/validate-package gripper_control
```

## 🎯 Integration

- Updates TODO.md with progress
- Validates prerequisites
- Generates next steps
- Updates system status

## 📖 Quality Gates

- Structure follows standards
- Build system configured correctly
- Code compiles without warnings
- Tests pass and coverage adequate
- Documentation complete and accurate
- Performance within baselines

---

_Validate package command for comprehensive quality assurance._
