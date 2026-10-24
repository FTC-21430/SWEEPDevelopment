package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;

/**
 * A break waypoint that marks a path continuity boundary at its target pose.
 * It is created by {@link com.broombots.sweep.Builder.SequenceBuilder#addBreak()}.
 */
public class BreakWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Pos2D coordinate;

	/**
	 * Creates a break waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint, with position in inches and heading in degrees
	 * @throws IllegalArgumentException if coordinate is null
	 */
	public BreakWaypoint(Pos2D coordinate) {
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
	 * @return zero because break waypoints do not own a movement speed
	 */
	@Override
	public double getSpeed() {
		return 0;
	}

	/**
	 * @return waypoint type token for path generation dispatch
	 */
	@Override
	public WaypointType getType() {
		return WaypointType.BREAK;
	}
}
