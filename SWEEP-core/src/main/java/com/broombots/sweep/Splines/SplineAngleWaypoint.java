package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;

/**
 * A path anchor waypoint that defines a target pose and movement mode for path generation.
 * This variant requests curved spline translation while holding the coordinate heading fixed.
 */
public class SplineAngleWaypoint implements Waypoint {
	/**
	 * Segment speed scale in the range [0, 1] relative to robot top speed.
	 */
	private final double speed;

	/**
	 * Target pose for this waypoint (x/y in inches, heading in degrees).
	 */
	private final Pos2D coordinate;

	/**
	 * Creates a spline-angle waypoint from primitive pose values.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param angle fixed heading in degrees
	 * @param speed segment speed scale relative to robot top speed
	 */
	public SplineAngleWaypoint(double x, double y, double angle, double speed) {
		this.coordinate = new Pos2D(x, y, angle);
		this.speed = speed;
	}

	/**
	 * Creates a spline-angle waypoint from an existing coordinate.
	 *
	 * @param coordinate target pose for this waypoint
	 * @param speed segment speed scale relative to robot top speed
	 */
	public SplineAngleWaypoint(Pos2D coordinate, double speed) {
		if (coordinate == null) throw new IllegalArgumentException("coordinate cannot be null");

		this.coordinate = coordinate;
		this.speed = speed;
	}

	/**
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
		return WaypointType.SPLINE_ANGLE;
	}
}
