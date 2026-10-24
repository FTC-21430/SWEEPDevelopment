package com.broombots.sweep.Classes;

/** Mutable two-dimensional position or vector with a heading/angular component. */
public class Pos2D {
    /** X component in inches for positions or inches per second for velocity vectors. */
    public double x;
    /** Y component in inches for positions or inches per second for velocity vectors. */
    public double y;
    /** Heading in degrees for poses or degrees per second for angular velocity. */
    public double angle;

    /** Creates a position or vector with the supplied components.
     * @param x x position in inches or vector component in inches per second
     * @param y y position in inches or vector component in inches per second
     * @param angle heading in degrees or angular velocity in degrees per second
     */
    public Pos2D(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
    /** Computes the Euclidean distance between two positions, ignoring their angles.
     * @param c1 first position in inches
     * @param c2 second position in inches
     * @return distance in inches
     */
    public static double getDistanceBetweenCoordinates(Pos2D c1, Pos2D c2){
        return Math.hypot(c2.x - c1.x, c2.y - c1.y);
    }
    /** Computes the magnitude of this vector's x/y components.
     * @return magnitude in inches for a position vector or inches per second for a velocity vector
     */
    public double getMagnitude(){
        return Math.hypot(x,y);
    }
    /** Rotates the x/y components about the origin, counterclockwise, without changing angle.
     * @param degrees counterclockwise rotation in degrees
     * @return a new rotated position or vector
     */
    public Pos2D rotateAroundZeroBy(double degrees){
        double translationAngle = Math.toRadians(degrees);
        double x = this.x * Math.cos(translationAngle) - this.y * Math.sin(translationAngle);
        double y = this.x * Math.sin(translationAngle) + this.y * Math.cos(translationAngle);
        return new Pos2D(x, y, this.angle);

    }
    /** Linearly interpolates all three components, clamping the ratio to [0, 1].
     * @param ratio interpolation ratio, clamped to the range [0, 1]
     * @param p1 starting position or vector
     * @param p2 ending position or vector
     * @return interpolated position or vector
     */
    public static Pos2D lerpPos2D(double ratio, Pos2D p1, Pos2D p2){
        return new Pos2D(
                lerp(p1.x,p2.x,ratio),
                lerp(p1.y,p2.y,ratio),
                lerp(p1.angle,p2.angle,ratio)
        );
    }
    /** Computes the direction from {@code p2} to {@code p1}.
     * @param p1 destination point of the vector, in inches
     * @param p2 origin point of the vector, in inches
     * @return direction from {@code p2} to {@code p1} in radians, in the range returned by {@link Math#atan2(double, double)}
     */
    public static double getMovementDirectionRad(Pos2D p1, Pos2D p2){
        return Math.atan2(p1.y - p2.y, p1.x - p2.x);
    }
    /** Interpolates between two scalar values using the supplied ratio. */
    private static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x);
        return start + (end-start) * xInRange;
    }

}
