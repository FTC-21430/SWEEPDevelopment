package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;

/**
 * A spline waypoint whose heading follows the direction of travel plus a constant offset.
 * The curve is the same as for a {@link SplineWaypoint}; an offset of 180 degrees drives the robot backwards.
 */
public class SplineOffsetWaypoint implements Waypoint {
	/** Segment speed scale in the range [0, 1] relative to robot top speed. */
	private final double speed;

	/** Target position for this waypoint, x/y in inches. The angle is unused, the offset sets the heading. */
	private final Pos2D coordinate;

	/** Angle added to the direction of travel, in degrees, positive is counterclockwise. */
	private final double headingOffset;

	/**
	 * Creates an offset spline waypoint.
	 *
	 * @param x target x position in inches
	 * @param y target y position in inches
	 * @param headingOffset angle added to the direction of travel, in degrees
	 * @param speed segment speed scale relative to robot top speed
	 */
	public SplineOffsetWaypoint(double x, double y, double headingOffset, double speed) {
		this.coordinate = new Pos2D(x, y, 0);
		this.headingOffset = headingOffset;
		this.speed = speed;
	}

	/** @return target pose for this waypoint */
	@Override
	public Pos2D getCoordinate() {
		return coordinate;
	}

	/** @return target x position in inches */
	@Override
	public double getX() {
		return coordinate.x;
	}

	/** @return target y position in inches */
	@Override
	public double getY() {
		return coordinate.y;
	}

	/** @return placeholder heading in degrees, the real heading comes from the direction of travel plus the offset */
	@Override
	public double getAngle() {
		return coordinate.angle;
	}

	/** @return segment speed scale relative to robot top speed */
	@Override
	public double getSpeed() {
		return speed;
	}

	/** @return angle added to the direction of travel, in degrees */
	@Override
	public double getHeadingOffset() {
		return headingOffset;
	}

	/** @return {@link Waypoint.WaypointType#SPLINE_OFFSET} */
	@Override
	public WaypointType getType() {
		return WaypointType.SPLINE_OFFSET;
	}
}
