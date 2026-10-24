package org.firstinspires.ftc.teamcode.ExampleCode;

import com.broombots.sweep.Builder.Sequence;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.broombots.sweep.Classes.SWEEPRobot;

/**
 * Base linear opmode for defining and running SWEEP autonomous routes.
 * Subclasses assign a sequence and robot in {@link #defineRoute()}.
 */
public abstract class SWEEPRoute extends LinearOpMode {
    /** Route sequence, public so subclasses can assign it and the base opmode can execute it. */
    public Sequence sequence;
    /** Robot executing the route, public so subclasses can assign it and the base opmode can initialize it. */
    public SWEEPRobot robot;
    /**
     * Runs the route lifecycle: defineRoute, initialize, wait for start, set the sequence, then update autonomously.
     * @throws InterruptedException if the opmode thread is interrupted while waiting for start
     */
    @Override
    public void runOpMode() throws InterruptedException {
        defineRoute();
        if (sequence == null) throw new RuntimeException("Path is not defined. Override defineRoute() method and create a path.");
        if (robot == null) throw new RuntimeException("Robot is not defined. Override defineRoute() method and create a robot.");
        robot.initialize();
        waitForStart();
        robot.setPath(sequence);
        while (opModeIsActive()) {
            robot.autonomousUpdate();
        }
    }
    /**
     * Defines the route sequence and assigns the robot that will execute it.
     * Subclasses should build the route with {@code SequenceBuilder} and assign the result to {@link #sequence}.
     */
    public void defineRoute() {
        robot = null;
        sequence = null;
    }
}
