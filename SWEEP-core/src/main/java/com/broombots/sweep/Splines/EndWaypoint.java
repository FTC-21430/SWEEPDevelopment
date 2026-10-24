package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;

/**
 * An end waypoint that marks the terminal pose of a path.
 * It is created by {@link com.broombots.sweep.Builder.SequenceBuilder#end(double, double, double)}; its speed value is
 * 1.0 so the profile can decelerate to a stop at the endpoint.
 */
public class EndWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Pos2D coordinate;

	/**
	 * Creates an end waypoint from primitive pose values.
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param angle target heading in degrees
	 */
	public EndWaypoint(double x, double y, double angle) {
		this.coordinate = new Pos2D(x, y, angle);
	}

	/**
	 * Creates an end waypoint from an existing coordinate.
	 * @param coordinate target pose for this waypoint, with position in inches and heading in degrees
	 * @throws IllegalArgumentException if coordinate is null
	 */
	public EndWaypoint(Pos2D coordinate) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");

		this.coordinate = coordinate;

	}

	/**
	 * @return target x position in inches
	 */
	@Override
	public double getX() {
		return coordinate.x;
	}

	/**
	 * @return target y position in inches
	 */
	@Override
	public double getY() {
		return coordinate.y;
	}

	/**
	 * Gets the target pose, with position in inches and heading in degrees.
	 *
	 * @return target pose for this waypoint
	 */
	@Override
	public Pos2D getCoordinate() {
		return coordinate;
	}

	/**
	 * @return target heading in degrees
	 */
	@Override
	public double getAngle() {
		return coordinate.angle;
	}

	/**
	 * @return segment speed scale relative to robot top speed.
	 * Returns 1.0 so the preceding segment's ideal velocity map is non-zero;
	 * comeToStop in MotionProfileProcessor decelerates the robot to a stop at this boundary.
	 */
	@Override
	public double getSpeed() {
		return 1.0;
	}

	/**
	 * @return waypoint type token for path generation dispatch
	 */
	@Override
	public WaypointType getType() {
		return WaypointType.END;
	}
}
