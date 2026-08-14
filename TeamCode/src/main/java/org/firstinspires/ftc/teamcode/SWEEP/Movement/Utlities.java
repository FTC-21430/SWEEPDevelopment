package org.firstinspires.ftc.teamcode.SWEEP.Movement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

//TODO Refactor this class and work on bringing up to date.
//NOTE THIS CLASS SHOULD NOT BE REVIEWED OR USED FOR ANYTHING OTHER THAN REFERENCE. IT IS NOT CURRENTLY FUNCTIONAL AND IS IN THE PROCESS OF BEING REWRITTEN.

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

