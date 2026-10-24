package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.broombots.sweep.Classes.Localization;
import com.broombots.sweep.Classes.LocalizationPacket;
/**
 * This class implements the Localization interface to provide localization functionality using the GoBilda Pinpoint Driver.
 * It handles the retrieval of the robot's position and orientation, as well as calculating velocities based on position changes over time.
 */
public class GobildaLocalization implements Localization {
    /** GoBilda Pinpoint device providing odometry and heading. */
    private final GoBildaPinpointDriver pinpoint;
    /** Previous loop's robot x position in inches. */
    private double lastRobotX;
    /** Previous loop's robot y position in inches. */
    private double lastRobotY;
    /** Current robot x position in inches. */
    private double robotX;
    /** Current robot y position in inches. */
    private double robotY;
    /** Current robot heading in degrees. */
    private double robotAngle;
    /** Current robot x velocity in inches/second. */
    private double velocityX;
    /** Current robot y velocity in inches/second. */
    private double velocityY;
    /** Seconds elapsed since the previous velocity update. */
    private double time;
    /** Telemetry destination for velocity readings. */
    private final Telemetry telemetry;
    /** Timer used to measure elapsed loop time in seconds. */
    private final ElapsedTime runtime;

    /**
     * Creates localization using the configured GoBilda Pinpoint and odometry pod offsets.
     * @param hardwareMap hardware map used to retrieve the Pinpoint device
     * @param telemetry telemetry destination for velocity readings
     * @param xPodOffset x-axis pod offset in inches, based on the robot's measured pod mounting geometry
     * @param yPodOffset y-axis pod offset in inches, based on the robot's measured pod mounting geometry
     * @param reset whether to reset the stored pose and recalibrate the IMU
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
     * Updates the robot's position, heading, and velocities from the GoBilda Pinpoint readings.
     */
    @Override
    public void update() {
        // Retain the previous pose so the next velocity estimate uses this loop's displacement.
        lastRobotX = robotX;
        lastRobotY = robotY;

        pinpoint.update();
        Pose2D position = pinpoint.getPosition();
        robotX = position.getX(DistanceUnit.INCH);
        robotY = position.getY(DistanceUnit.INCH);
        robotAngle = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        updateVelocity();
    }
    /**
     * Returns the current robot pose and velocity estimate.
     * @return localization packet with positions in inches, heading in degrees, and velocities in inches/second
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
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, angle));
    }
    /**
     * Measures and resets the elapsed time since the previous call.
     * @return elapsed time in seconds
     */
    private double getDeltaTime(){
        time = runtime.seconds();
        runtime.reset();
        return time;
    }
    /**
     * Updates velocity estimates from the current and previous poses; called from {@link #update()}.
     */
    private void updateVelocity(){
        double dt = getDeltaTime();
        velocityX = (robotX - lastRobotX)/dt;
        velocityY = (robotY - lastRobotY)/dt;
        telemetry.addData("velocityX", velocityX);
        telemetry.addData("velocityY", velocityY);
    }
    /** Recalibrates the Pinpoint's inertial measurement unit. */
    public void calibrate(){
        pinpoint.recalibrateIMU();
    }

}
