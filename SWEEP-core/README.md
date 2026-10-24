# SWEEP-core

SWEEP-core is BroomBots FTC Team 21430's Java library for building and following
time-parameterized robot paths. This is an early beta: its APIs and motion
profiling are still evolving, so validate paths on blocks and at reduced speed
before competition use.

## Design and execution flow

The path pipeline separates geometry, timing, and hardware:

1. `SequenceBuilder` collects typed waypoints and scheduled `SWEEPAction`s.
2. `PathProcessor` converts waypoints into spline segments, samples the
   geometry, calculates a translational velocity profile, and assigns times.
3. `Sequence` stores the compiled `PathPoint`s and checks action trigger
   conditions as the path runs.
4. `SequenceInterpreter` selects the current target, updates actions, and uses
   `RobotMovementController` to calculate robot-relative drive powers.
5. A robot-specific `SWEEPRobot` updates localization and sends those powers to
   the drivetrain during each OpMode loop.

This split lets path geometry be planned without embedding motor or sensor
details in the path builder. SWEEP provides shared interfaces and example
defaults; the robot project remains responsible for accurate localization,
hardware setup, and tuning.

## Units and coordinate conventions

Use inches for length, degrees for angles, and seconds for time. Translational
velocities use inches per second; angular velocities use degrees per second.
`SequenceBuilder.setSampleRate()` is an exception to the time convention: its
value is a dimensionless step through the spline parameter from 0 to 1, not a
time interval. A smaller step creates more samples and takes longer to compile.

`Pos2D` carries x, y, and an angle. For poses the angle is a heading; for velocity
vectors it is angular velocity. `SequenceInterpreter.update()` returns
robot-relative power values: x is lateral, y is forward, and angle is rotation
power. Confirm that your drivetrain uses the same axis and sign conventions.

## Creating a sequence

Supply motion limits, then add a start, one or more waypoints, and an end:

```java
RobotMovementParameters limits = new DefaultRobotMovementParameters();
Sequence route = new SequenceBuilder(limits)
        .start(0, 0, 0)
        .splineTo(24, 12, 1.0)
        .splineToBackwards(36, 24, 0.75)
        .waitAt(0.5)
        .splineToOffset(48, 24, 90, 0.6)
        .end(60, 12, 0)
        .build();
```

Builder methods chain and return the same builder. Speed arguments are ratios of
the configured maximum speed and are clipped to `[0, 1]`. Coordinates are in
inches and are clipped to the current field bounds. The `Pos2D` overloads are
useful when a pose is already available; for offset splines, its angle is
ignored.

### Waypoint behavior

| Builder method | Robot heading | Behavior |
| --- | --- | --- |
| `start(x, y, heading)` | Explicit starting heading | Must be first. |
| `splineTo(x, y, speed)` | Direction of travel | Follows the spline tangent. |
| `splineToAngle(x, y, heading, speed)` | Transitions to the requested heading | Uses an angled spline segment. |
| `splineToOffset(x, y, offset, speed)` | Travel direction plus offset | Positive offset is counterclockwise. |
| `splineToBackwards(x, y, speed)` | Travel direction plus 180 degrees | Drives with the rear leading. |
| `addBreak()` | Maintains the surrounding waypoint heading behavior | Stops at a path break before continuing. |
| `waitAt(seconds)` | Holds the current pose | Stops for the requested duration. |
| `end(x, y, heading)` | Explicit ending heading | Must terminate the sequence. |

`splineTo`, `splineToAngle`, and `splineToOffset` are spline waypoints. The
builder maps them to `FollowSplineSegment` or `AngledSplineSegment`; `waitAt`
maps to a stationary `WaitSegment`. `start`, `end`, and `addBreak` delimit or
split the movement geometry. Linear waypoint enum values are not currently
implemented as builder methods.

An offset is added to the tangent angle, so raw segment headings can lie outside
`[-180, 180]` degrees. During compilation `PathProcessor` unwraps each heading
to the nearest equivalent of the previous heading. This avoids an artificial
full-turn discontinuity when a tangent crosses the `atan2` branch boundary.
Changing abruptly between forward and backwards offsets still requests a real
180-degree turn; use a break or wait at that transition if the robot should
rotate while stopped.

## Running a sequence

Each robot must supply hardware-backed localization and a drivetrain. A
`SWEEPRobot` subclass can start the route with `setPath(route)`. In its active
loop, update localization, get the current `LocalizationPacket`, call
`sequenceInterpreter.update(packet)`, and send the returned x/y/rotation powers
to the drivetrain. The loop should remain non-blocking so the FTC OpMode can
stop promptly.

The repository's [TeamCode examples](../TeamCode/README.md) show this wiring in
`TemplateBot`, `SWEEPRoute`, and `StraightLineTest`.

## Path profiling and important interactions

`PathProcessor` samples each movement segment in spline-parameter space. It
estimates local curvature from the absolute change in travel direction divided
by the average distance between adjacent samples. A hand-tuned interpolation
table turns that estimate into a speed scale; the segment's own speed ratio and
the robot's directional maximum speed further limit the local speed goal.

The forward velocity pass uses the constant-acceleration relationship
`v^2 = u^2 + 2as` to cap each sample at the speed reachable from the previous
sample. The backward pass applies the same constraint from the terminal speed
toward the start, limiting speeds where braking is needed. Time is then
integrated between adjacent samples using distance divided by their average
translational speed. These discrete approximations depend on the sample rate
and configured movement limits.

Waits split the route into movement runs. Each run before a wait is planned to
end at zero speed, the wait contributes stationary zero-velocity samples, and
the following run starts from that held pose and heading. Segment-time markers
let action scheduling distinguish progress through the route from merely being
near a trigger coordinate.

Angular velocity is not yet included in the output `PathPoint` velocity
profile. The current movement controller separately calculates heading error
and rotation power while following the translational profile.

### Action triggers and lifecycle

`SequenceBuilder.addAction(action)` queues a `SWEEPAction`. If the action has no
explicit position, the builder associates it with the previous waypoint. An
autonomous action starts only when:

- the robot is within the action's planar trigger radius (heading is not part
  of the distance check);
- the configured wait time has elapsed on the sequence clock; and
- the waypoint marker associated with the action has been reached.

The sequence executes one queued action at a time. `execute()` runs once when
the action starts, `process()` runs on active updates, `completion()` determines
natural completion, and `end()` runs once through the guarded `finish()`
lifecycle. Accepted interruption can keep an action processing while it
recovers to a safe state; `isInterrupted()` and the optional recovery timeout
support mechanism-specific cleanup.

For example, implement a mechanism action by extending `SWEEPAction` and
overriding `execute`, `process`, `completion`, and `end`. See
[`ExampleAction`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleCode/ExampleAction.java)
for a compilable template.

### Appending a route

`Sequence.addToSequence(builder)` and
`SequenceInterpreter.addToCurrentSequence(builder)` continue from the current
sequence's last pose, velocity, and time. The appended builder's own start is
ignored, and action waypoint indices are shifted to the combined route. The
interpreter method requires an active sequence; call `startPath()` first.

## Movement parameters and drivetrain model

`RobotMovementParameters` supplies directional maximum translation speed and
acceleration, angular limits, a heading threshold, and a 3x3 PID matrix. The
matrix rows correspond to position, rotation, and velocity controllers; the
columns are P, I, and D. `DefaultRobotMovementParameters` is a generic starting
point, not a tuned configuration. The dashboard-tunable parameters in TeamCode
are intended for on-robot experimentation.

`MecanumDriveAcceleration` provides a directional factor for its mecanum
assumptions. Real motor behavior, wheel traction, battery voltage, robot mass,
localization quality, and drivetrain mapping affect actual performance. Validate
the configured limits and controller gains on the physical robot.

## Comparison and tradeoffs

These are architectural comparisons, not performance rankings; select and tune
a path library against your robot and season requirements.

| Library | Documented approach | SWEEP tradeoff |
| --- | --- | --- |
| SWEEP | Team-owned Java API with chained waypoints, sampled spline paths, motion limits, and an integrated action scheduler. | Easy to adapt to this team's robot abstractions; early beta maturity means fewer integrations, tuning tools, and field-proven edge cases. |
| [Road Runner](https://rr.brott.dev/docs/v1-0/actions/) | Its 1.0 documentation describes composable cooperative actions; its tuning guide documents robot-specific calibration steps. | Offers an established action composition model and a substantial tuning workflow. SWEEP instead keeps a direct waypoint builder and team-specific robot lifecycle. |
| [Pedro Pathing](https://pedropathing.com/docs) | Its project documentation describes an FTC path follower using Bézier curves and disturbance correction. | Provides a dedicated path-following approach; SWEEP uses its own sampled spline/profile pipeline and does not claim equivalent disturbance rejection or maturity. |

SWEEP is a fit when owning and modifying the path and action implementation is
valuable. Prefer an established library when its integration ecosystem,
documentation, tuning support, or demonstrated robustness is more important
than maintaining a team-specific implementation.

## Glossary

- **SWEEP:** the team's path planning, following, and action-scheduling library.
- **Waypoint:** a typed route instruction that the builder converts to path
  geometry or a stationary interval.
- **Spline parameter:** normalized curve input `t` in `[0, 1]`; it is not time.
- **Catmull-Rom:** the cubic interpolation used to form a curve through the
  route's control points.
- **Path point:** one timestamped pose and velocity sample in a compiled
  sequence.
- **OpMode:** the FTC SDK program lifecycle used to run autonomous or teleop
  behavior.

## Build and tests

From the repository root:

```sh
./gradlew :SWEEP-core:testDebugUnitTest
./gradlew :TeamCode:testDebugUnitTest
./gradlew :TeamCode:compileDebugJavaWithJavac
```

The project targets Java 8 and is built as part of the FTC Android Gradle
project.
