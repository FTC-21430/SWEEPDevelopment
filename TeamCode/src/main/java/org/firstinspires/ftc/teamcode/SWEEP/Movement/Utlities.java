package org.firstinspires.ftc.teamcode.SWEEP.Movement;

/**
 * General utility methods for the SWEEP package.
 */
public class Utlities {

    /**
     * Wraps an angle into the range (-180, 180].
     * @param angle the angle to wrap (degrees)
     * @return the equivalent angle in (-180, 180]
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

