package com.broombots.sweep.Classes;

/** One time-stamped position and velocity sample in a compiled sequence. */
public class PathPoint {
    /** Seconds elapsed since the sequence started. */
    public double time;
    /** Robot pose: x and y in inches and heading in degrees. */
    public Pos2D position;
    /** Robot velocity: x and y in inches per second and angular velocity in degrees per second. */
    public Pos2D velocity;

}
