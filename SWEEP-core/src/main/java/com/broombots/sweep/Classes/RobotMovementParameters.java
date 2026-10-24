package com.broombots.sweep.Classes;

import org.ejml.simple.SimpleMatrix;

/** Supplies physical motion limits and PID gains used when building and following robot paths. */
public interface RobotMovementParameters {
    /** Returns the maximum translational speed for travel in a field direction.
     * @param direction travel direction in degrees
     * @return maximum speed in inches per second
     */
    public double getMaxVelocity(double direction);
    /** Returns the maximum robot heading speed.
     * @return maximum angular speed in degrees per second
     */
    public double getMaxAngleVelocity();

    /** Returns the maximum stable translational acceleration for a direction.
     * @param direction travel direction in degrees
     * @return maximum acceleration in inches per second squared
     */
    public double getMaxStableAcceleration(double direction);
    /** Returns the maximum stable angular acceleration at the requested translation speed and turn direction.
     * @param movementMagnitude translational speed in inches per second
     * @param clockwise {@code true} for clockwise rotation, {@code false} for counterclockwise
     * @return maximum angular acceleration in degrees per second squared
     */
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise);
    /** Returns the heading error threshold below which the controller applies its full rotation power.
     * @return angular error threshold in degrees
     */
    public double getAngleFullPowerToErrorThreshold();
    /**
     * Returns PID gains in a 3x3 matrix: rows 0, 1, and 2 are position, heading, and velocity controllers;
     * columns 0, 1, and 2 are proportional, integral, and derivative gains. Position gains are shared by x/y,
     * and velocity gains are shared by vx/vy.
     * @return 3x3 PID gain matrix, with gain units determined by each controller's input and output units
     */
    public SimpleMatrix getPIDCoefficients();
}
