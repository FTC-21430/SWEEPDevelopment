package org.firstinspires.ftc.teamcode.SWEEP.Classes;

/**
 * Represents a 2D position and heading on the field.
 * Used throughout SWEEP as the fundamental unit of robot pose.
 */
public class Coordinate {
    /** X and Y positions in inches, and heading angle in degrees. */
    private final double x, y, angle;
    /**
     * Constructs a new Coordinate with the given position and heading.
     * @param x     X position, in inches
     * @param y     Y position, in inches
     * @param angle Heading angle, in degrees
     */
    public Coordinate(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
    /**
     * Simplified version of Coordinate that only requires unique values for x and y
     * @param x
     * @param y
     */
    public Coordinate(double x, double y){
        this.x = x;
        this.y = y;
        this.angle = 0;
    }
    /** @return X position, in inches. */
    public double getX() {
        return x;
    }

    /** @return Y position, in inches. */
    public double getY() {
        return y;
    }

    /** @return Heading angle, in degrees. */
    public double getAngle() {
        return angle;
    }
    /**
     * Computes the Euclidean distance between two coordinates, ignoring heading.
     * @param c1 First coordinate
     * @param c2 Second coordinate
     * @return Distance between {@code c1} and {@code c2}, in inches
     */
    public static double getDistanceBetweenCoordinates(Coordinate c1, Coordinate c2){
        return Math.hypot(c2.getX() - c1.getX(), c2.getY() - c1.getY());
    }
}
