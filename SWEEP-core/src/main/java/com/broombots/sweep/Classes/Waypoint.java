package com.broombots.sweep.Classes;

/** Describes a position and movement instruction used to build a robot path. */
public interface Waypoint {
    /** Kinds of waypoint that determine how sequence construction treats a point. */
    public enum WaypointType {
        /** Cubic spline whose robot heading follows travel direction. */
        SPLINE,
        /** Cubic spline with heading smoothly directed toward the waypoint angle. */
        SPLINE_ANGLE,
        /** Cubic spline with heading offset from travel direction. */
        SPLINE_OFFSET,
        /** Reserved linear-motion waypoint type; not yet supported. */
        LINEAR,
        /** Reserved linear-motion waypoint type with fixed heading; not yet supported. */
        LINEAR_ANGLE,
        /** Stops at the point and separates adjacent path segments. */
        BREAK,
        /** Holds the robot at the point for the waypoint duration. */
        WAIT,
        /** Marks the beginning of a path; does not create a segment by itself. */
        START,
        /** Marks the terminal path position and causes the path to stop there. */
        END
    }

    /** Returns the waypoint's field X coordinate.
     * @return X position in inches */
    public double getX();
    /** Returns the waypoint's field Y coordinate.
     * @return Y position in inches */
    public double getY();
    /** Returns the robot heading associated with this waypoint.
     * @return heading in degrees */
    public double getAngle();
    /** Returns the waypoint's field pose.
     * @return position in inches and heading in degrees */
    public Pos2D getCoordinate();

    /** Returns the requested speed ratio, if specified.
     * @return speed as a fraction of the robot's maximum speed, defaulting to zero */
    default public double getSpeed(){
        return 0;
    }
    /** Returns how long the robot waits at this waypoint; only wait waypoints have non-zero duration.
     * @return wait duration in seconds, defaulting to zero */
    default public double getDuration(){
        return 0;
    }
    /** Returns the heading offset for spline-offset waypoints.
     * @return offset from travel direction in degrees, positive counterclockwise; zero by default */
    default public double getHeadingOffset(){
        return 0;
    }
    /** Returns the movement or control behavior represented by this waypoint.
     * @return this waypoint's type */
    public WaypointType getType();
}
