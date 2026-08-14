package org.firstinspires.ftc.teamcode.SWEEP.Splines;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint;

/**
 * A path anchor waypoint that defines a target pose and movement mode for path generation.
 * This variant pauses execution at the target pose for a configured duration.
 */
public class WaitWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Coordinate coordinate;

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
		this(new Coordinate(x, y, angle), duration);
	}

	/**
	 * Creates a wait waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 * @param duration hold duration in seconds
	 */
	public WaitWaypoint(Coordinate coordinate, double duration) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");

		this.coordinate = coordinate;
		this.duration = duration;
	}

	/**
	 * @return target pose for this waypoint
	 */
	@Override
	public Coordinate getCoordinate() {
		return coordinate;
	}

	/**
	 * @return target x position in inches
	 */
	@Override
	public double getX() {
		return coordinate.getX();
	}

	/**
	 * @return target y position in inches
	 */
	@Override
	public double getY() {
		return coordinate.getY();
	}

	/**
	 * @return target heading in degrees
	 */
	@Override
	public double getAngle() {
		return coordinate.getAngle();
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
