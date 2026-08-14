package org.firstinspires.ftc.teamcode.SWEEP.Splines;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint;

/**
 * A path anchor waypoint that defines a target pose and movement mode for path generation.
 * This variant requests linear translation while holding the coordinate heading as a fixed angle.
 */
public class LinearAngleWaypoint implements Waypoint {
	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Coordinate coordinate;

	/**
	 * Segment speed scale in the range [0, 1] relative to robot top speed.
	 */
	private final double speed;

	/**
	 * Creates a linear-angle waypoint from primitive pose values.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param angle fixed heading in degrees
	 * @param speed segment speed scale relative to robot top speed
	 */
	public LinearAngleWaypoint(double x, double y, double angle, double speed) {
		this.coordinate = new Coordinate(x, y, angle);
		this.speed = speed;
	}

	/**
	 * Creates a linear-angle waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 * @param speed segment speed scale relative to robot top speed
	 */
	public LinearAngleWaypoint(Coordinate coordinate, double speed) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");

		this.coordinate = coordinate;
		this.speed = speed;
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
	 * @return segment speed scale relative to robot top speed
	 */
	@Override
	public double getSpeed() {
		return speed;
	}

	/**
	 * @return waypoint type token for path generation dispatch
	 */
	@Override
	public WaypointType getType() {
		return WaypointType.LINEAR_ANGLE;
	}
}
