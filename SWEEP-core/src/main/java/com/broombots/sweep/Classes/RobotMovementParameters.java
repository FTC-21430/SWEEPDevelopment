package com.broombots.sweep.Classes;

public interface RobotMovementParameters {
    // Max velocity parameters
    public double getMaxVelocity(double direction);

    // Max Acceleration parameters
    public double getMaxStableAcceleration(double direction, double angleError);
    public double getMaxStableAngularAcceleration(double rotation);

    // Min Acceleration parameters
    public double getMinAcceleration(double direction);
}
