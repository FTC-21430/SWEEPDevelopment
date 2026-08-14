package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Localization;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.LocalizationPacket;
/**
 * This class implements the Localization interface to provide localization functionality using the GoBilda Pinpoint Driver.
 * It handles the retrieval of the robot's position and orientation, as well as calculating velocities based on position changes over time.
 */
public class GobildaLocalization implements Localization {
    // variable location for stored pinpoint from hardware map
    private final GoBildaPinpointDriver pinpoint;
    private double lastRobotX, lastRobotY;

    //    stored x and y and angle components
    private double robotX, robotY; // in inches
    private double robotAngle; // in degrees
    // These are the current velocities of the robot, updates each iteration.
    private double velocityX, velocityY;
    private double time;
    private Telemetry telemetry;
    private ElapsedTime runtime;

    /**
     * Constructor for this class.
     * @param hardwareMap used to get the module from hardware map
     * @param xPodOffset configures the pinpoint for how we have set it up to ensure current results -CM
     * @param yPodOffset configures the pinpoint for how we have set it up to ensure current results -CM
     * @param reset should we reset and recalibrate the sensor and its positions / angles? false for transition between auto and teleop to avoid transition error (non-fatal, just difference of value)
     */
    public GobildaLocalization(HardwareMap hardwareMap, Telemetry telemetry, double xPodOffset, double yPodOffset, boolean reset){

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(xPodOffset,yPodOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        if (reset) {
            pinpoint.resetPosAndIMU();
            pinpoint.recalibrateIMU();
        }

        this.telemetry = telemetry;

        runtime = new ElapsedTime();
    }

    /**
     * Updates the robot's position, orientation, and velocities based on the current readings from the GoBilda Pinpoint Driver.
     */
    @Override
    public void update() {
        // store the last iterations values so we can calculate the velocities!
            lastRobotX= robotX;
            lastRobotY = robotY;

            pinpoint.update();
            Pose2D position =  pinpoint.getPosition();
        //        Our frame of reference and the gobilda frame of reference is different, so you units needed to change.
            robotX = position.getX(DistanceUnit.INCH);
            robotY = position.getY(DistanceUnit.INCH);
            robotAngle = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
            updateVelocity();
    }
    /**
     * Returns a LocalizationPacket containing the current position, orientation, and velocities of the robot.
     * @return LocalizationPacket with current robot state
     */
    @Override
    public LocalizationPacket getLocalizationPacket() {
        return new LocalizationPacket(robotX, robotY, robotAngle, velocityX, velocityY);
    }
    /**
     * Overrides the current localization values with the provided x, y, and angle.
     * @param x The new x position in inches
     * @param y The new y position in inches
     * @param angle The new orientation angle in degrees
     */
    @Override
    public void overrideLocalization(double x, double y, double angle) {
        robotX = x;
        robotY = y;
        robotAngle = angle;
    }
    /**
     * Returns the time between loop iterations - resets between calls!
     * @return The time in seconds since the last call
     */
    private double getDeltaTime(){
        time = runtime.seconds();
        runtime.reset();
        return time;
    }
    /**
     * Updates the velocityX and velocityY variables based new robot variables. MUST CALL WITHIN updateOdometry()!
     */
    private void updateVelocity(){
        double dt = getDeltaTime();
        velocityX = (robotX - lastRobotX)/dt;
        velocityY = (robotY - lastRobotY)/dt;
        telemetry.addData("velocityX", velocityX);
        telemetry.addData("velocityY", velocityY);
    }

}
