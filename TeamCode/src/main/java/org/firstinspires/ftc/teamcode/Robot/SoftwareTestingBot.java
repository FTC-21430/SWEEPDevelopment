package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ExampleCode.ExampleAction;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.SWEEPRobot;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
/**
 * This class represents a software testing robot that extends the SWEEPRobot class.
 * It contains subsystems and actions specific to the software testing robot.
 */
public class SoftwareTestingBot extends SWEEPRobot {
    // Robot subsystems
    Drivetrain drivetrain;
    /**
     * Constructor for the SoftwareTestingBot class.
     * Initializes the robot's subsystems and takes FIRSTSDK bits
     * @param hardwareMap The hardware map used to access the robot's hardware components.
     * @param telemetry   The telemetry object used for sending data to the driver station.
     */
    public SoftwareTestingBot(HardwareMap hardwareMap, Telemetry telemetry){
        runtime = new ElapsedTime();
        this.drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        subsystems = new Subsystem[]{
                drivetrain,
        };
    }
    /**
     * Initializes the robot's subsystems and sets up any necessary configurations.
     * This method is called once when the robot is initialized.
     */
    @Override
    public void initialize() {
        drivetrain.enableFieldCentricDriving(false);
    }
    /**
     * Updates the robot's subsystems and actions during teleop mode.
     * This method is called repeatedly during the teleop period.
     */
    @Override
    public void teleopUpdate() {
        updateActions();
    }
    /**
     * Updates the robot's subsystems and actions during autonomous mode.
     * This method is called repeatedly during the autonomous period.
     */
    @Override
    public void autonomousUpdate() {
        updateActions();
    }
    // Subsystem getters
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    // Robot action definitions
    /**
     * Creates an ExampleAction to demonstrate how actions should be defined
     * @param coordinate The coordinate around which the action is triggered.
     * @param time       The duration for which the action runs (in seconds).
     * @param range      The trigger radius for the action (in inches).
     * @return An instance of ExampleAction.
     */
    public ExampleAction exampleAction(Coordinate coordinate, double time, double range){
        return new ExampleAction(this, coordinate, time, range);
    }

    /**
     * Simplified example action which used default parameters
     * @return new exampleAction
     */
    public ExampleAction exampleAction(){
        return new ExampleAction(this, null, getDefaultActionTime(), getDefaultActionRange());
    }
}
