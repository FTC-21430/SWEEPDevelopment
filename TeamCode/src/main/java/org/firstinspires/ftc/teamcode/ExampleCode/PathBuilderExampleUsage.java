package org.firstinspires.ftc.teamcode.ExampleCode;
import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.TemplateBot;

/**
 * Example usage of {@link SequenceBuilder} to create a sequence for the robot to follow.
 * This class demonstrates building a sequence with movements, waits, and actions.
 */
@Disabled // remove if you are going to copy and use this code.
// Labels the route in the Driver Station menu. Change the name and group to your liking.
@Autonomous(name = "SequenceBuilder Example Usage", group = "ExampleCode")
public class PathBuilderExampleUsage extends SWEEPRoute {
    /** Robot instance used to configure this example sequence. */
    TemplateBot r; // Software testing bot is the example here, r is just shorthand for robot.
    /** Configures the example robot and its movement-and-action sequence. */
    @Override
    public void defineRoute() {
        r = new TemplateBot(hardwareMap, telemetry); // create the new robot object, passing FIRSTSDK bits to the constructor.
        super.robot = r; // Assigning the your robot the the superclasses robot, which it will then use to run the route on.
        super.sequence = new SequenceBuilder(new DefaultRobotMovementParameters()) // create the sequence that will be executed.
                .start(0, 0, 0) // All paths must have a start point as the first
                .splineTo(10, 10, 1) // robot will move in a spline path toward end point from wherever the last point was (start), robot heading will follow the curve of the path
                .splineToAngle(20, 20, 90, 1) // robot will move in a spline path toward end point from wherever the last point was, robot heading will follow the curve of the path and end at the specified angle
                .addBreak() // adds a break in the path, robot will slow down and stop at the point, but immediately continue to the next point in the path
                .addAction(r.exampleAction()) // adds an action to be executed at the current point in the path
                .waitAt(2) // robot will wait at the current point for the specified duration (in seconds)
                .end(0, 40, 0) // robot will move to the end point with the specified coordinates and heading
                .build(); // finalizes the path and returns a sequence

    }
}
