package org.firstinspires.ftc.teamcode.SWEEP.Runtime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.SoftwareTestingBot;
import org.firstinspires.ftc.teamcode.SWEEP.Builder.Path;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.SWEEPRobot;

/**
 * Abstract class designed for creating new autonomous routes with SWEEP
 */
public abstract class SWEEPRoute extends LinearOpMode {
    public Path path;
    public SWEEPRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        defineRoute();
        if (path == null) throw new RuntimeException("Path is not defined. Override defineRoute() method and create a path.");
        if (robot == null) throw new RuntimeException("Robot is not defined. Override defineRoute() method and create a robot.");
        robot.initialize();
        waitForStart();
        robot.setPath(path);
        while (opModeIsActive()) {
            robot.autonomousUpdate();
        }
    }
    /**
     * This method should be overridden in subclasses to define the specific route for the robot.
     * The route should be defined using the PathBuilder class and assigned to the 'path' variable.
     */
    public void defineRoute() {
        robot = null;
        path = null;
    }
}
