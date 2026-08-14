package org.firstinspires.ftc.teamcode.ExampleCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot.SoftwareTestingBot;
import org.firstinspires.ftc.teamcode.SWEEP.Builder.PathBuilder;
import org.firstinspires.ftc.teamcode.SWEEP.Runtime.SWEEPRoute;

@Disabled // remove if you are going to copy and use this code.
// Labels the route in the Driver Station menu. Change the name and group to your liking.
@Autonomous(name = "PathBuilder Example Usage", group = "ExampleCode")
/**
 * Example usage of the PathBuilder class to create a path for the robot to follow.
 * This class demonstrates how to build a path with various movements and actions.
 */
public class PathBuilderExampleUsage extends SWEEPRoute {
    // The specific robot you will use for the route. Change this to your robot class.
    SoftwareTestingBot r; // Software testing bot is the example here, r is just shorthand for robot.
    @Override
    public void defineRoute() {
        r = new SoftwareTestingBot(hardwareMap, telemetry); // create the new robot object, passing FIRSTSDK bits to the constructor.
        super.robot = r; // Assigning the your robot the the superclasses robot, which it will then use to run the route on.
        super.path = new PathBuilder() // create the path that will be executed.
                .start(0, 0, 0) // All paths must have a start point as the first
                .splineTo(10, 10, 0.5) // robot will move in a spline path toward end point from wherever the last point was (start), robot heading will follow the curve of the path
                .splineToAngle(20, 20, 90, 0.5) // robot will move in a spline path toward end point from wherever the last point was, robot heading will follow the curve of the path and end at the specified angle
                .addBreak() // adds a break in the path, robot will slow down and stop at the point, but immediately continue to the next point in the path
                .linearTo(30, 30, 0.5) // robot will move in a straight line toward end point from wherever the last point was
                .linearToAngle(40, 40, 180, 0.5) // robot will move in a straight line toward end point from wherever the last point was, robot heading will end at the specified angle
                .addAction(r.exampleAction()) // adds an action to be executed at the current point in the path
                .waitAt(2) // robot will wait at the current point for the specified duration (in seconds)
                .end(60, 60, 270, 0.5) // robot will move to the end point with the specified coordinates and heading
                .build(); // finalizes the path and returns a Path object

    }
}
