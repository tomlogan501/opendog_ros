# Validate Architecture Command —

## 🎯 Purpose

Validate Clean Architecture compliance.

## 🚀 Usage

```bash
/validate-architecture [package_name]
```

## 📋 What it validates (code-free checks)

- Business logic separated from frameworks
- No circular dependencies
- Proper layer isolation and boundaries
- Interface contracts defined and documented
- Dependency inversion applied

## 📋 Architecture Layers

- **Domain Layer**: Pure business logic
- **Use Cases Layer**: Application rules
- **Adapters Layer**: Interface conversion
- **Frameworks Layer**: Integration only

## 📤 Output

- Architecture compliance report
- Layer isolation validation
- Dependency analysis
- Recommendations for improvement

## 🔍 Examples

```bash
/validate-architecture turret_control
/validate-architecture pump_control
/validate-architecture gripper_control
```

## 🎯 Integration

- Updates TODO.md with progress
- Validates prerequisites
- Generates next steps
- Updates system status

## 📖 Quality Gates

- Clean Architecture principles followed
- Layer isolation maintained
- Interface contracts defined
- Dependency inversion applied
- No circular dependencies

---

_Validate architecture command for Clean Architecture compliance._
