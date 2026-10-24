package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.MecanumDriveAcceleration;

import org.ejml.simple.SimpleMatrix;

/**
 * Dashboard-tunable movement parameters for on-robot tuning.
 * This implementation provides editable values in place of SWEEP's {@code DefaultRobotMovementParameters}.
 */
@Config
public class DebuggingRobotMovementParameters implements RobotMovementParameters {
    /** Robot mass in kilograms retained for the mass-taking constructor. */
    private final double robotMass;
    /** Public static so this value can be tuned live from FTC Dashboard; average force per wheel in newtons. */
    public static double averageMotorForce = 8.5;
    /** Public static so this value can be tuned live from FTC Dashboard; maximum linear acceleration in inches/s^2. */
    public static double robotTopAcceleration = 25;
    /** Public static so this value can be tuned live from FTC Dashboard; maximum angular acceleration in degrees/s^2. */
    public static double robotTopAngularAcceleration = 10;
    /** Public static so this value can be tuned live from FTC Dashboard; maximum angular velocity in degrees/s. */
    public static double robotTopAngularVelocity = 30;
    /** Public static so this value can be tuned live from FTC Dashboard; maximum straight-line speed in inches/s. */
    public static double robotTopSpeedStraight = 40;
    /** Public static so this value can be tuned live from FTC Dashboard; full-turn duration in seconds. */
    public static double timeForFullTurn = 1.2;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless position proportional coefficient. */
    public static double standardPositionP = 0.95;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless position integral coefficient. */
    public static double standardPositionI = 0.12;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless position derivative coefficient. */
    public static double standardPositionD = 0.01;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless rotation proportional coefficient. */
    public static double rotationP = 0.07;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless rotation integral coefficient. */
    public static double rotationI = 0.02;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless rotation derivative coefficient. */
    public static double rotationD = 0.0005;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless velocity proportional coefficient. */
    public static double velocityP = 0.09;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless velocity integral coefficient. */
    public static double velocityI = 0.004;
    /** Public static so this gain can be tuned live from FTC Dashboard; unitless velocity derivative coefficient. */
    public static double velocityD = 0.001;

    /**
     * Creates movement parameters using the supplied robot mass.
     * @param robotMass robot mass in kilograms
     */
    public DebuggingRobotMovementParameters(double robotMass){
        this.robotMass = robotMass;
    }
    /** Creates movement parameters using the default robot mass of 10 kilograms. */
    public DebuggingRobotMovementParameters(){
        robotMass = 10; // Kg for a light weight FTC robot - Should be tuned
    }

    /**
     * @param direction movement direction in degrees
     * @return maximum linear speed in inches/s
     */
    public double getMaxVelocity(double direction){
        return (robotTopSpeedStraight) * MecanumDriveAcceleration.getMovementMagnitude(direction) * (Math.sqrt(2)/2);
    }
    /**
     * @return maximum angular speed in degrees/s
     */
    public double getMaxAngleVelocity(){
        return robotTopAngularVelocity; // degrees per second
    }
    /**
     * @param direction movement direction in degrees
     * @return maximum stable acceleration in inches/s^2
     */
    public double getMaxStableAcceleration(double direction){
        return robotTopAcceleration * MecanumDriveAcceleration.getMovementMagnitude(direction);
    }
    /**
     * Returns the configured maximum stable angular acceleration.
     * @param movementMagnitude requested movement magnitude
     * @param clockwise whether the requested rotation is clockwise
     * @return maximum angular acceleration in degrees/s^2
     */
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise){
        return robotTopAngularAcceleration; // degrees/s^2
    }
    /**
     * @return angular error threshold for applying full rotation power, in degrees
     */
    @Override
    public double getAngleFullPowerToErrorThreshold() {
        return 40;
    }


    /**
     * Returns the proportional, integral, and derivative gains for position, rotation, and velocity.
     * @return a 3-by-3 coefficient matrix; gains are unitless
     */
    @Override
    public SimpleMatrix getPIDCoefficients(){
        SimpleMatrix coeffs = new SimpleMatrix(3,3);
        coeffs.set(0,0, standardPositionP);
        coeffs.set(0,1,standardPositionI);
        coeffs.set(0,2,standardPositionD);

        coeffs.set(1,0, rotationP);
        coeffs.set(1,1,rotationI);
        coeffs.set(1,2,rotationD);

        coeffs.set(2,0,velocityP);
        coeffs.set(2,1,velocityI);
        coeffs.set(2,2,velocityD);

        return coeffs;
    }
}
