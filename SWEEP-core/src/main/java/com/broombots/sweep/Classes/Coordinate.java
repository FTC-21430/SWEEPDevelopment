package com.broombots.sweep.Classes;

/** Immutable field position and heading used to identify robot poses. */
public class Coordinate {
    /** X and Y field positions in inches, and heading angle in degrees. */
    private final double x, y, angle;
    /** Constructs a coordinate with the given position and heading.
     * @param x field X position in inches
     * @param y field Y position in inches
     * @param angle heading in degrees
     */
    public Coordinate(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
    /** Constructs a coordinate with a zero-degree heading.
     * @param x field X position in inches
     * @param y field Y position in inches
     */
    public Coordinate(double x, double y){
        this.x = x;
        this.y = y;
        this.angle = 0;
    }
    /** @return X position in inches */
    public double getX() {
        return x;
    }

    /** @return Y position in inches */
    public double getY() {
        return y;
    }

    /** @return heading in degrees */
    public double getAngle() {
        return angle;
    }
    /** Computes Euclidean distance between two coordinates, ignoring heading.
     * @param c1 first coordinate
     * @param c2 second coordinate
     * @return distance between the coordinates in inches
     */
    public static double getDistanceBetweenCoordinates(Coordinate c1, Coordinate c2){
        return Math.hypot(c2.getX() - c1.getX(), c2.getY() - c1.getY());
    }
}
