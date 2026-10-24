package com.broombots.sweep.Movement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Provides small shared movement math utilities.
 * Maintainer note: this class is scheduled for a refactor, so its API may change before 1.0.
 */
@Disabled
public class Utilities {

    /**
     * Wraps an angle into the range [-180, 180].
     * @param angle the angle to wrap (degrees)
     * @return the equivalent angle in [-180, 180] degrees
     */
    public static double wrap(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}
