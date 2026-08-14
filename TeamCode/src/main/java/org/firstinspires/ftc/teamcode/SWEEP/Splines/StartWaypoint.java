package org.firstinspires.ftc.teamcode.SWEEP.Splines;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint;

/**
 * A path anchor waypoint that defines a target pose and movement mode for path generation.
 * This variant marks the robot's starting pose before any generated segment begins.
 */
public class StartWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Coordinate coordinate;

	/**
	 * Creates a start waypoint from primitive pose values.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param angle target heading in degrees
	 */
	public StartWaypoint(double x, double y, double angle) {
		this.coordinate = new Coordinate(x, y, angle);
	}

	/**
	 * Creates a start waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 */
	public StartWaypoint(Coordinate coordinate) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");
		this.coordinate = coordinate;
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
	 * @return waypoint type token for path generation dispatch
	 */
	@Override
	public WaypointType getType() {
		return WaypointType.START;
	}
}
