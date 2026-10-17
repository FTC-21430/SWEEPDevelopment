package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.MecanumDriveAcceleration;

import org.ejml.simple.SimpleMatrix;

@Config
public class DebuggingRobotMovementParameters implements RobotMovementParameters {
    double robotMass;
    public static double averageMotorForce = 8.5; // N Per Wheel on ground;
    public static double robotTopAcceleration = 25; // inches/s^2
    public static double robotTopAngularAcceleration = 10; //degrees/s^2
    public static double robotTopAngularVelocity = 30; //degrees/second
    public static double robotTopSpeedStraight = 40; // inches/s
    public static double timeForFullTurn = 1.2; // Seconds
    public static double standardPositionP = 0.95;
    public static double standardPositionI = 0.12;
    public static double standardPositionD = 0.01;
    public static double rotationP = 0.12;
    public static double rotationI = 0.02;
    public static double rotationD = 0.0;
    public static double velocityP = 0.09;
    public static double velocityI = 0.004;
    public static double velocityD = 0.001;

    public DebuggingRobotMovementParameters(double robotMass){
        this.robotMass = robotMass;
    }
    public DebuggingRobotMovementParameters(){
        robotMass = 10; // Kg for a light weight FTC robot - Should be tuned
    }

    public double getMaxVelocity(double direction){
        // 1.5 m/s for the average robot
        return (robotTopSpeedStraight) * MecanumDriveAcceleration.getMovementMagnitude(direction) * (Math.sqrt(2)/2);
    }
    public double getMaxAngleVelocity(){
        return robotTopAngularVelocity; // degrees per second
    }
    public double getMaxStableAcceleration(double direction){
        return robotTopAcceleration * MecanumDriveAcceleration.getMovementMagnitude(direction);
    }
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise){
        return robotTopAngularAcceleration; // degrees / s^2 Simple values similar to what road runners defaults are. I will tune a basic chassis to this and make those values the default later.
    }
    private double getAverageIndividualWheelAcceleration(){
        double metersAccel = averageMotorForce / robotMass; // N / Kg = m / s^2
        return metersAccel * (100.0/2.45);
    }

    @Override
    public double getAngleFullPowerToErrorThreshold() {
        return 40;
    }


    /** coefficient matrix in form
     * xy : p i d
     * angle : p i d
     * vX X : p i d
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
