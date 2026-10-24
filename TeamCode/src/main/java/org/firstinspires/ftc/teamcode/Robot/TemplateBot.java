package org.firstinspires.ftc.teamcode.Robot;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Movement.SequenceInterpreter;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ExampleCode.ExampleAction;
import com.broombots.sweep.Classes.SWEEPRobot;
import com.broombots.sweep.Classes.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.GobildaLocalization;

/**
 * Robot configuration for the team's template FTC robot.
 * It wires the drivetrain, GoBilda localization, movement parameters, and example actions into SWEEP.
 */
public class TemplateBot extends SWEEPRobot {
    /** Drivetrain subsystem used for robot movement. */
    Drivetrain drivetrain;
    /** GoBilda Pinpoint localization subsystem. */
    GobildaLocalization gobildaLocalization;
    /** GoBilda Pinpoint y pod offset in inches; public static for live FTC Dashboard tuning. */
    public static double yOffset = 2.78;
    /** GoBilda Pinpoint x pod offset in inches; public static for live FTC Dashboard tuning. */
    public static double xOffset = 4.9574;
    /**
     * Creates the template robot and its hardware-backed subsystems.
     * Localization is assigned to the inherited field in {@link #initialize()} after it is constructed.
     * @param hardwareMap hardware map used to access robot hardware
     * @param telemetry telemetry destination for the driver station
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
     * Initializes the drivetrain and makes the constructed localization subsystem active.
     * This method is called once after construction and before the opmode starts.
     */
    @Override
    public void initialize() {
        drivetrain.enableFieldCentricDriving(true);
        localization = gobildaLocalization;
        localization.overrideLocalization(0,0,0);
    }
    /**
     * Updates drivetrain and actions during teleop, then flushes telemetry.
     */
    @Override
    public void teleopUpdate() {
        drivetrain.update();
        updateActions();
        telemetry.update();
    }
    /**
     * Updates localization, autonomous movement, drivetrain, and telemetry for one loop.
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

    /**
     * Returns the robot's drivetrain subsystem.
     * @return configured drivetrain
     */
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    /**
     * Creates an example action with explicit trigger and timing parameters.
     * @param position trigger center in inches
     * @param time action duration in seconds
     * @param range trigger radius in inches
     * @param waitTime minimum elapsed path time before triggering, in seconds
     * @return configured example action
     */
    public ExampleAction exampleAction(Pos2D position, double time, double range, double waitTime){
        return new ExampleAction(this, position, time, range, waitTime);
    }

    /**
     * Creates an example action using the robot's default trigger settings.
     * @return configured example action
     */
    public ExampleAction exampleAction(){
        return new ExampleAction(this, null, getDefaultActionTime(), getDefaultActionRange(), 0);
    }
}
