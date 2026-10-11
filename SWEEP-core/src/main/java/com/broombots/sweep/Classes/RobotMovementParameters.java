package com.broombots.sweep.Classes;

import org.ejml.simple.SimpleMatrix;

public interface RobotMovementParameters {
    // Max velocity parameters
    public double getMaxVelocity(double direction, double angleError);
    public double getMaxAngleVelocity();

    // Max Acceleration parameters
    public double getMaxStableAcceleration(double direction, double angleError);
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise);
    public double getAngleFullPowerToErrorThreshold();
    /** coefficient matrix in form
     * xy : p i d
     * angle : p i d
     * vX X : p i d

     */
    public SimpleMatrix getPIDCoefficients();
}
