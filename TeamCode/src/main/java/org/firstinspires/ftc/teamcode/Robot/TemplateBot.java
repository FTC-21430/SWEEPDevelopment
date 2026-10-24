package org.firstinspires.ftc.teamcode.Robot;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.broombots.sweep.Movement.SequenceInterpreter;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ExampleCode.ExampleAction;
import com.broombots.sweep.Classes.SWEEPRobot;
import com.broombots.sweep.Classes.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.GobildaLocalization;

import com.broombots.sweep.Classes.Coordinate;
/**
 * This class represents a software testing robot that extends the SWEEPRobot class.
 * It contains subsystems and actions specific to the software testing robot.
 */
public class TemplateBot extends SWEEPRobot {
    // Robot subsystems
    Drivetrain drivetrain;
    GobildaLocalization gobildaLocalization;
    public static double yOffset = 2.78;
    public static double xOffset = 4.9574;
    /**
     * Constructor for the SoftwareTestingBot class.
     * Initializes the robot's subsystems and takes FIRSTSDK bits
     * @param hardwareMap The hardware map used to access the robot's hardware components.
     * @param telemetry   The telemetry object used for sending data to the driver station.
     */
    public TemplateBot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        robotMovementParameters = new DebuggingRobotMovementParameters();
        sequenceInterpreter = new SequenceInterpreter(robotMovementParameters);
        super.localization = gobildaLocalization;
        runtime = new ElapsedTime();
        this.drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        subsystems = new Subsystem[]{
                drivetrain,
        };
        gobildaLocalization = new GobildaLocalization(hardwareMap,telemetry,xOffset,yOffset,true);
        gobildaLocalization.calibrate();
    }
    /**
     * Initializes the robot's subsystems and sets up any necessary configurations.
     * This method is called once when the robot is initialized.
     */
    @Override
    public void initialize() {
        drivetrain.enableFieldCentricDriving(true);
        localization = gobildaLocalization;
        localization.overrideLocalization(0,0,0);
    }
    /**
     * Updates the robot's subsystems and actions during teleop mode.
     * This method is called repeatedly during the teleop period.
     */
    @Override
    public void teleopUpdate() {
        drivetrain.update();
        updateActions();
        telemetry.update();
    }
    /**
     * Updates the robot's subsystems and actions during autonomous mode.
     * This method is called repeatedly during the autonomous period.
     */
    @Override
    public void autonomousUpdate() {
        updateActions();
        localization.update();
        Pos2D resultingPowers = new Pos2D(0,0,0);
        if (sequenceInterpreter.hasActiveSequence()){
            resultingPowers = sequenceInterpreter.update(localization.getLocalizationPacket());
        }
        telemetry.addData("power x", resultingPowers.x);
        telemetry.addData("power y", resultingPowers.y);
        telemetry.addData("power angle", resultingPowers.angle);
        drivetrain.updateMovementVector(resultingPowers.x, resultingPowers.y, resultingPowers.angle);
        drivetrain.update();
        //Other sweep logic
        telemetry.update();
    }

    // Subsystem getters
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    // Robot action definitions
    /**
     * Creates an ExampleAction to demonstrate how actions should be defined
     * @param position The coordinate around which the action is triggered.
     * @param time       The duration for which the action runs (in seconds).
     * @param range      The trigger radius for the action (in inches).
     * @param waitTime   The path time (in seconds) that must pass before the action can trigger.
     * @return An instance of ExampleAction.
     */
    public ExampleAction exampleAction(Pos2D position, double time, double range, double waitTime){
        return new ExampleAction(this, position, time, range, waitTime);
    }

    /**
     * Simplified example action which used default parameters
     * @return new exampleAction
     */
    public ExampleAction exampleAction(){
        return new ExampleAction(this, null, getDefaultActionTime(), getDefaultActionRange(), 0);
    }
}
