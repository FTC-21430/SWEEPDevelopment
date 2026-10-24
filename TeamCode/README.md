# TeamCode examples

`TeamCode` is the FTC robot application module in this repository. It depends on
the local `SWEEP-core` Gradle module and contains an example hardware
integration, route and actions. These examples demonstrate structure; hardware
names, offsets, localization settings, and motion parameters must be checked
against the actual robot before use.

## How the example is wired

- `Robot/TemplateBot` constructs the drivetrain and GoBilda Pinpoint
  localization subsystem, selects movement parameters, and connects them to a
  `SequenceInterpreter`.
- `Robot/Subsystems/Drivetrain` maps robot-relative movement powers onto the
  four configured mecanum motors. Its configured motor names are `fl`, `fr`,
  `bl`, and `br`.
- `Robot/Subsystems/GobildaLocalization` reads robot pose and velocity and
  publishes a SWEEP `LocalizationPacket`.
- `ExampleCode/ExampleAction` shows the action lifecycle for a mechanism action.
- `ExampleCode/SWEEPRoute` is a base `LinearOpMode` that calls `defineRoute`,
  initializes the robot, starts its sequence, and repeatedly updates the robot.
- `ExampleCode/PathBuilderExampleUsage` demonstrates chained waypoints, a
  heading-controlled spline, a break, an action, and a wait. It is disabled by
  default as a template.
- `Opmodes/autonomous/StraightLineTest` demonstrates a route that drives the
  drivetrain and reports target and localization telemetry.

## Adapting the template

1. Create a robot subclass or adapt `TemplateBot` to match your hardware
   configuration. Set correct motor names, directions, encoder behavior,
   localization parameters, and initialization sequence.
2. Tune a `RobotMovementParameters` implementation with conservative values.
   Confirm all lengths are inches, angles are degrees, and times are seconds.
3. Create an `@Autonomous` OpMode derived from `SWEEPRoute` (or write an
   equivalent explicit loop). In `defineRoute()`, construct your robot and
   assign both the robot and a built `Sequence` to the base route fields.
4. Test localization and drivetrain directions independently, then run a short
   path at low speed with a clear stop path. Increase limits only after
   verifying tracking and braking behavior on the physical robot.
5. Implement mechanism behavior in a `SWEEPAction` subclass. Keep `process()`
   non-blocking; use `end()` for cleanup and check `isInterrupted()` when an
   interrupted action needs a different safe-state behavior.

Offset splines can drive rear-first:

```java
builder
        .splineToBackwards(24, 12, 0.5)
        .addBreak()
        .splineToOffset(36, 12, 90, 0.5);
```

Changing between forward and backwards headings requests a real 180-degree
orientation change. The break in the example stops at the transition so the
robot can turn while stationary. See the [SWEEP-core guide](../SWEEP-core/README.md)
for path semantics, action scheduling, and known limitations.
