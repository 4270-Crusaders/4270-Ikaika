# 4270 FRC Java Coding Standards
---
## Abstract

This document establishes the 4270 FRC Java Coding Standards, a comprehensive framework designed to ensure software reliability, maintainability, and high-performance execution for robotic systems. Developed for the unique constraints of the FIRST Robotics Competition (FRC) environment, these standards integrate modern Java 21+ features with industry-standard "Clean Code" principles and robotics-specific safety protocols.
The core of this standard centers on a Hardware Abstraction Layer (HAL) architecture, utilizing interfaces to decouple subsystem logic from specific motor controller implementations. Key pillars include:
Self-Documenting Code: Strict naming conventions and mandatory unit-suffixes (e.g., targetAngleDegrees) to eliminate mechanical integration errors.
Predictable Execution: Guardrails for WPILib periodic methods to prevent loop overruns and ensure real-time responsiveness. 

Collaborative Integrity: A structured Git workflow requiring multi-tier peer reviews and mentor oversight to maintain a stable, competition-ready codebase.
Safety & Defense: Rigid requirements for error logging, hardware initialization checks, and the total elimination of "magic numbers" and unhandled exceptions.
By adopting these practices, Team 4270 aims to bridge the gap between student learning and professional engineering, producing code that is not only robust under the physical rigors of competition but also easily accessible for rapid iteration and multi-year knowledge transfer.

---

### 1. Naming Conventions
Variables and Methods: Use camelCase and ensure names are descriptive yet concise (no longer than 4 words).
Example: 
```java 
	int motorSpeed;
```

Classes: Begin with a capital letter.
Example: 
```java 
	class MotorController
```

Enums: Use ALL_CAPS.
Example: 
```java 
	enum MOTOR_TYPE { 
		NEO, FALCON 
	}
```

Constants: Use ALL_CAPS for private static finals.
Example: 
```java 
	private static final int MAX_SPEED = 100;
```

### 2. Code Structure
Subsystems: Use an interface for the subsystem (e.g., SubsystemIO.java), the real subsystem class (e.g., SubsystemName.java), and the motorController IO (e.g., SubsystemIONeo.java or SubsystemIOPhoenix.java).

### 3. Git Standards
- Branch Naming: Branches should be named according to the subsystem and feature being developed (e.g., drive-system-enhancements).
- Commit Messages: Keep them short and descriptive (e.g., "Fixed encoder reading bug").

#### Branch Management:
- Create branches per subsystem or significant changes.
- A dedicated competition branch should be maintained.
- All changes must be pull requested into the main branch and approved by a mentor and/or lead programmer (at least 2 approvals for students, 1 approval from a mentor).

### 4. Programming Best Practices
#### Defensive Coding
- Avoid Nulls: Use Optional<T> for return types that might be empty.
- Immutable by Default: Use final for variables that do not need to change. Prefer using record classes for data-only objects.
- Exceptions: Never leave a catch block empty. At the very least, log the error.
#### The "Rule of One"
- Single Responsibility: A class should do one thing. Break down large classes into smaller, focused ones.
- Method Length: If a method exceeds 20 lines, consider refactoring it into smaller helper methods.

### 5. Documentation & Comments
- Javadoc: Every public and protected member should have a Javadoc comment.
- Comment the "Why": Explain why the code does something, not what it does.
Example:
```java
// Offset for the header margin
```
- Clean Up: Delete commented-out code. Use Git versioning for reference.

### 6. Modern Java Features (2024–2026)
- Switch Expressions: Use yield or the arrow -> syntax for more readable logic.
- Text Blocks: Use """ for multi-line strings to avoid messy concatenation.
- Streams: Use the Stream API for collection processing where it improves readability.

### 7. Units of Measure
- Standardize Units: Explicitly state the units in variable names for physical quantities.
Examples:
```java
	double targetAngleDegrees;
	double velocityMetersPerSecond;
```

### 8. WPILib & Hardware Specifics
- Periodic Methods: Avoid heavy calculations or blocking calls inside teleopPeriodic() or subsystemPeriodic().
- Telemetry/SmartDashboard: Use structured naming for keys (e.g., Subsystem/ValueName).

### 9. The "Constants" File
- Organize Constants: Use inner classes to group constants by subsystem.
```java
	public final class Constants {
		public static final class DriveConstants {
			public static final int LEFT_MOTOR_ID = 1;
		}
	}
```

### 10. Safety & Error Handling
- Logging: Use DataLogManager or a custom logger. Avoid System.out.println as it's expensive.
- Hardware Initialization: Check for hardware presence during robotInit.

### 11. Logic & Mathematics
- Math Tooling: Use MathUtil.clamp() and WPILib’s Units library for conversions.
- Magic Numbers: Forbid magic numbers. Use named constants instead.

---
By adhering to these standards, our FRC team can ensure that our code is clean, efficient, and easy to maintain, aligning with practices used by major tech companies.