package com.broombots.sweep.Classes;

public interface RobotMovementParameters {
    // Max velocity parameters
    public double getMaxVelocity(double direction, double angleError);
    public double getAngleVelocity();

    // Max Acceleration parameters
    public double getMaxStableAcceleration(double direction, double angleError);
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise);
    public double getAngleFullPowerToErrorThreshold();
}
