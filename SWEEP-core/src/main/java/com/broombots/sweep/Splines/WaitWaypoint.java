package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;

/**
 * A wait waypoint that pauses at its target pose for a configured duration in seconds.
 * It is created by {@link com.broombots.sweep.Builder.SequenceBuilder#waitAt(double)}.
 */
public class WaitWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Pos2D coordinate;

	/**
	 * Time to hold at this waypoint in seconds.
	 */
	private final double duration;

	/**
	 * Creates a wait waypoint from primitive pose values.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param angle target heading in degrees
	 * @param duration hold duration in seconds
	 */
	public WaitWaypoint(double x, double y, double angle, double duration) {
		this(new Pos2D(x, y, angle), duration);
	}

	/**
	 * Creates a wait waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 * @param duration hold duration in seconds
	 * @throws IllegalArgumentException if coordinate is null
	 */
	public WaitWaypoint(Pos2D coordinate, double duration) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");

		this.coordinate = coordinate;
		this.duration = duration;
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
	 * @return hold duration in seconds
	 */
	@Override
	public double getDuration() {
		return duration;
	}

	/**
	 * @return waypoint type token for path generation dispatch
	 */
	@Override
	public WaypointType getType() {
		return WaypointType.WAIT;
	}
}
