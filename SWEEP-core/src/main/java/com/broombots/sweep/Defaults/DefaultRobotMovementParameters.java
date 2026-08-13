package com.broombots.sweep.Defaults;

import com.broombots.sweep.Classes.RobotMovementParameters;

public class DefaultRobotMovementParameters implements RobotMovementParameters {
    double robotMass;
    private final double averageMotorForce = 8.5; // N Per Wheel on ground;
    private final double robotTopSpeedStraight = 1.5; // m/s
    private final double timeForFullTurn = 1.2; // Seconds
    public DefaultRobotMovementParameters(double robotMass){
        this.robotMass = robotMass;
    }
    public DefaultRobotMovementParameters(){
        robotMass = 10; // Kg for a light weight FTC robot - Should be tuned
    }

    public double getMaxVelocity(double direction, double angleError){
        // 1.5 m/s for the average robot
        return (1.5 / 2) * MecanumDriveAcceleration.getMovementMagnitude(direction, angleError);
    }
    public double getAngleVelocity(){
        return 360/timeForFullTurn; // degrees per second
    }
    public double getMaxStableAcceleration(double direction, double angleError){
        return getAverageIndividualWheelAcceleration() * MecanumDriveAcceleration.getMovementMagnitude(direction, angleError);
    }
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise){
        return 200; // degrees / s^2 Simple values similar to what road runners defaults are. I will tune a basic chassis to this and make those values the default later.
    }
    private double getAverageIndividualWheelAcceleration(){
        return averageMotorForce / robotMass; // N / Kg = m / s^2
    }
}
