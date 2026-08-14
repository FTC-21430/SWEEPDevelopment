# SWEEP Codebase — Copilot Style Guide

This is a Java FTC (FIRST Tech Challenge) robotics codebase. Follow these conventions when generating or editing code.

---

## Project Structure

| Package | Purpose |
|---|---|
| `SWEEP.Classes` | Shared data types and interfaces (`Waypoint`, `Action`, `Coordinate`, etc.) |
| `SWEEP.Splines` | Waypoint implementations and spline math (`CubicSplineSegment`, `SplineWaypoint`, etc.) |
| `SWEEP.Builder` | Path construction and planning (`PathPlanning`, `PathBuilder`, `GlobalPositions`) |
| `SWEEP.Movement` | Drivetrain PID and motion control (`AccelerationControl`, `RotationControl`) |
| `SWEEP.Runtime` | Path execution and action scheduling (`SplinePathInterpreter`, `ActionManager`) |

New waypoint types always go in `SWEEP.Splines` and implement the `Waypoint` interface.

---

## Naming Conventions

- **Classes / Interfaces / Enums**: `PascalCase` — e.g. `SplinePathInterpreter`, `WaypointType`
- **Methods and fields**: `camelCase` — e.g. `getRobotPosition`, `currentSplineIndex`
- **Enum values**: `UPPER_SNAKE_CASE` — e.g. `SPLINE_ANGLE`, `CLOSE_START`
- **Constants** (`static final` primitives): `UPPER_SNAKE_CASE` — e.g. `MIN_HOLD_TIME`
- **Dashboard-exposed statics** (public, mutable): `camelCase` — e.g. `programSpeed`, `lookAheadTime1`
- Do **not** use Hungarian notation or type prefixes

---

## Formatting

- **Braces**: Opening brace on the same line (K&R style). 
  ```java
  public void myMethod(){
      // method body
  }
  ```
- **Indentation**: 1 tab per level
- **Blank lines**:
  - One blank line between methods
  - One blank line between the field block and the first constructor
  - No trailing blank lines inside method bodies
- **Line length**: Aim for under 120 characters; wrap long chains or argument lists
- **Imports**: Explicit imports only — no wildcard (`import java.util.*`) imports

---

## Fields and Declarations

- Prefer `private final` for fields that are set in the constructor and never reassigned
- Declare all fields at the top of the class, before constructors
- Group related fields on one line only when they share the same type and logical purpose (e.g. `private double x, y, angle`)
- `public static` fields are reserved for FTC Dashboard-exposed tuning variables and must have a comment explaining why they are public
- **All class-scope fields must have a Javadoc comment** (`/** */`) describing what the field represents and its units where applicable (e.g. inches, seconds, degrees)

---

## Methods

- **Guard clauses first**: Check for null, empty, or invalid inputs at the top of the method and return early
  ```java
  if (path == null || path.length == 0) return new SimpleMatrix(new double[]{0, 0, 0});
  ```
- Prefer ternary expressions for simple clamping or fallback assignments:
  ```java
  this.programSpeed = programSpeedDEBUG > 0.0 ? programSpeedDEBUG : 0.1;
  ```
- Builder methods return `this` to support chaining
- Keep methods focused; extract private helpers rather than letting a method grow beyond ~50 lines

---

## Comments and Javadoc

- **All public classes** get a Javadoc block explaining what the class does and its role in the system
- **All public methods** get a Javadoc block with `@param` tags for non-obvious parameters and a `@return` tag when the return value is non-trivial
- **Private helpers** get a short `//` comment above them if their purpose isn't self-evident from the name
- Inline `//` comments explain *why*, not *what* — avoid restating the code
- Use `// TODO:` for known incomplete sections with a brief description
- Attribution headers (copyright, collaborator notes) go at the very top of the file, before the `package` statement, using `///` line comments matching the existing style in `CubicSplineSegment.java`
- make blank java doc comments above even blank classes with ``//TODO:`` if the comment is not yet written 
---

## Enums

- Enums live as nested `static` types inside the class that owns them, unless they are shared across the package — in which case they belong in `SWEEP.Classes`
- The `WaypointType` enum is nested inside the `Waypoint` interface; use `Waypoint.WaypointType` when referencing it from outside

---

## Control Flow

- Use traditional `switch` statements with explicit `case`/`break` — do not use Java 14+ switch expressions unless the entire codebase targets API 34+
- Prefer `switch` on enums over chains of `if/else instanceof` checks
- Avoid deeply nested conditionals — flatten with early returns

---

## Waypoint System

- All waypoint types implement `org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint`
- The type of movement is identified by `getType()` returning a `Waypoint.WaypointType` enum value
- `shouldHoldAngle()` and `isWaitPoint()` are `default` methods on the interface — do **not** override them in implementations; they are derived automatically from `getType()`
- `generatePath()` in `PathPlanning` dispatches segment construction with a `switch` on `waypoint.getType()`
- `WaitWaypoint` is the only type where `getDuration()` returns a non-zero value

---

## FTC-Specific Notes

- `public static` fields used as FTC Dashboard variables must remain `public static` — do not refactor them to instance fields
- `ElapsedTime` objects are always `final` and constructed in the constructor, never reassigned
- Telemetry calls go in the lowest-level class that has access to the `Telemetry` reference — do not pass telemetry up through layers just to log

## Helping the target user
- Ensure that you explain the reasoning behind suggestions
- Provide context for why a particular approach or pattern is recommended
- Limit the length of suggestions to simple, easy parts. do not simply program everything for them, instead just start in one place and clarify if they want you to change classes outside of the active scope
- Your user enjoys programming themselves, and is competent, so allow them to do the heavy lifting and do simple things as they ask
