# Test Command —

## 🎯 Purpose

Validate implementation with comprehensive testing.

## 🚀 Usage

```bash
/test [package_name] [type]
```

## 📋 Test Types

- `unit` - Business logic tests
- `integration` - ROS2 interface tests
- `e2e` - End-to-end system tests
- `all` - Complete test suite

## 📋 What it does

1. Runs unit tests for business logic
2. Executes integration tests
3. Performs E2E testing
4. Generates coverage reports
5. Validates performance

## 📤 Output

- Test results and coverage reports
- Performance metrics
- Validation status
- Recommendations for improvements

## 🔍 Examples

```bash
/test gripper_control unit
/test gripper_control integration
/test gripper_control e2e
/test gripper_control all
```

## 🎯 Integration

- Updates TODO.md with progress
- Validates prerequisites
- Generates next steps
- Updates system status

## 📖 Quality Gates

- All tests pass
- Coverage adequate
- Performance validated
- System behavior verified

---

_Test command for comprehensive validation._
