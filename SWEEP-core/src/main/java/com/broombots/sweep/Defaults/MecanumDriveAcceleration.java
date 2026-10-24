package com.broombots.sweep.Defaults;

/** Provides the directional magnitude factor for a mecanum drivetrain. */
public class MecanumDriveAcceleration {
    /**
     * Computes {@code (sqrt(2) + 1) + (sqrt(2) - 1) * cos(4 * radians(movementDirection))}.
     * At 0 and 90 degrees the result is {@code 2 * sqrt(2)}; at 45 degrees it is 2.
     * @param movementDirection direction of travel in degrees
     * @return dimensionless directional magnitude factor
     */
    public static double getMovementMagnitude(double movementDirection){
        return (Math.sqrt(2) + 1) + (Math.sqrt(2) - 1) * Math.cos(4 * Math.toRadians(movementDirection));
    }
}
