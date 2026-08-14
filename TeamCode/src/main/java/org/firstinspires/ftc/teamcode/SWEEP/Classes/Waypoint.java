package org.firstinspires.ftc.teamcode.SWEEP.Classes;
/**
 * Waypoint interface, which is implemented by all waypoints in the robot's path.
 * Each waypoint must implement the getX(), getY(), getAngle(), getCoordinate(), getSpeed(), getDuration(), and getType() methods,
 * which are used to define the waypoint's position and behavior within the path.
 */
public interface Waypoint {
    /**
     * The Waypoint enum defines the type of the waypoint which will change the behavior of path generation around this point.
     * Some waypoints don't actually create new segments, but will serve other purposes, such as waiting at a point or breaking the path into multiple segments.
     */
    public enum WaypointType {
        SPLINE,
        SPLINE_ANGLE,
        LINEAR,
        LINEAR_ANGLE,
        BREAK,
        WAIT,
        START,
        END
    }

    /**
     * Gets the X coordinate of the waypoint.
     * @return
     */
    public double getX();
    /**
     * Gets the Y coordinate of the waypoint.
     * @return
     */
    public double getY();
    /**
     * Gets the heading angle of the waypoint.
     * @return
     */
    public double getAngle();
    /**
     * Gets the Coordinate object representing the waypoint's position and heading.
     * @return
     */
    public Coordinate getCoordinate();

    /**
     * Gets the speed at which the robot should move to this waypoint. If the waypoint does not require a specific speed, it returns 0 by default.
     * @return the robot speed in terms of speed/max speed
     */
    default public double getSpeed(){
        return 0; // If waypoint does not need speed, don't bother overriding it!
    }
    /**
     * Gets the duration for which the robot should wait at this waypoint. If the waypoint does not require waiting, it returns 0 by default.
     * @return the duration in seconds
     */
    default public double getDuration(){
        return 0; // Segment with no waiting at a single point
    }
    /**
     * Gets the type of the waypoint, which defines its behavior in the path.
     * @return the WaypointType of this waypoint
     */
    public WaypointType getType();
}
