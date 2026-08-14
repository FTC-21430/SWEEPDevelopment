package org.firstinspires.ftc.teamcode.SWEEP.Splines;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint;

/**
 * A path anchor waypoint that defines a target pose and movement mode for path generation.
 * This variant requests curved spline translation where heading may follow path direction.
 */
public class SplineWaypoint implements Waypoint {
	/**
	 * Segment speed scale in the range [0, 1] relative to robot top speed.
	 */
	private final double speed;

	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Coordinate coordinate;

	/**
	 * Creates a spline waypoint from primitive pose values.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param speed segment speed scale relative to robot top speed
	 */
	public SplineWaypoint(double x, double y, double speed) {
		this.coordinate = new Coordinate(x, y, 0);
		this.speed = speed;
	}

	/**
	 * Creates a spline waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 * @param speed segment speed scale relative to robot top speed
	 */
	public SplineWaypoint(Coordinate coordinate, double speed) {
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
		return WaypointType.SPLINE;
	}
}
