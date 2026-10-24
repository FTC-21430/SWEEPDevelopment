package com.broombots.sweep.Defaults;

public class MecanumDriveAcceleration {
    /**
     * represents the oscillating curve that is one mecanum wheel worth of power
     * SystemForce(theta) = (sqrt2 + 1) + (sqrt2 - 1)Cos(4 * theta)
     * for values theta
     * 0 = 2sqrt2
     * PI/2 = 2
     * PI = 2sqrt2
     * 3PI/2 = 2
     * 2PI = 2sqrt3
     * etc...
     */
    public static double getMovementMagnitude(double movementDirection){
        return (Math.sqrt(2) + 1) + (Math.sqrt(2) - 1) * Math.cos(4 * Math.toRadians(movementDirection));
    }
}