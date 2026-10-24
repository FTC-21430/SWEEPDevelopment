package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;

/**
 * A start waypoint that marks the robot's initial pose before any generated segment begins.
 * It is created by {@link com.broombots.sweep.Builder.SequenceBuilder#start(double, double, double)}.
 */
public class StartWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Pos2D coordinate;

	/**
	 * Creates a start waypoint from primitive pose values.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param angle target heading in degrees
	 */
	public StartWaypoint(double x, double y, double angle) {
		this.coordinate = new Pos2D(x, y, angle);
	}

	/**
	 * Creates a start waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 * @throws IllegalArgumentException if coordinate is null
	 */
	public StartWaypoint(Pos2D coordinate) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");
		this.coordinate = coordinate;
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
	 * @return target heading in degrees
	 */
	@Override
	public double getAngle() {
		return coordinate.angle;
	}

	/**
	 * @return waypoint type token for path generation dispatch
	 */
	@Override
	public WaypointType getType() {
		return WaypointType.START;
	}
}
