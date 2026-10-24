package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.SplineWaypoint;

import org.ejml.simple.SimpleMatrix;

/**
 * Placeholder segment for direct spline-follow logic that is not yet implemented.
 */
public class FollowSplineSegment implements Segment {
	CatmullRomCubic xCubic;
	CatmullRomCubic yCubic;
	private final double sampleRate = 0.001; // units
	private final double speed;

	/** Constant angle added to the direction of travel to get the robot heading, in degrees. */
	private final double headingOffsetDegrees;

	/**
	 * Creates a following spline segment whose heading is exactly the direction of travel.
	 *
	 * @param p1 control point before the start of this segment
	 * @param p2 where this segment starts
	 * @param p3 where this segment ends, its speed sets the speed of the segment
	 * @param p4 control point after the end of this segment
	 */
	public FollowSplineSegment(Waypoint p1, Waypoint p2, Waypoint p3, Waypoint p4) {
		this(p1, p2, p3, p4, 0);
	}

	/**
	 * Creates a following spline segment whose heading is the direction of travel plus a constant offset.
	 * An offset of 180 drives the robot backwards along the curve.
	 *
	 * @param p1 control point before the start of this segment
	 * @param p2 where this segment starts
	 * @param p3 where this segment ends, its speed sets the speed of the segment
	 * @param p4 control point after the end of this segment
	 * @param headingOffsetDegrees angle added to the direction of travel, in degrees, positive is counterclockwise
	 */
	public FollowSplineSegment(Waypoint p1, Waypoint p2, Waypoint p3, Waypoint p4, double headingOffsetDegrees) {
		this.headingOffsetDegrees = headingOffsetDegrees;
		xCubic = new CatmullRomCubic(p1.getX(), p2.getX(), p3.getX(), p4.getX());
		yCubic = new CatmullRomCubic(p1.getY(), p2.getY(), p3.getY(), p4.getY());
		// comeToStop in MotionProfileProcessor.compileVelocityProfile handles deceleration to zero
		// at BREAK/WAIT boundaries — forcing speed=0 here zeroed the entire ideal map, not just the endpoint.
		speed = p3.getSpeed();
	}
	/**
	 * @param time absolute time ( must range between 0-1 )
	 * @return pose at the requested time
	 */
	@Override
	public Pos2D getPosition(double time) {
		return new Pos2D(xCubic.evaluate(time), yCubic.evaluate(time), Math.toDegrees(Math.atan2(yCubic.derivative(time), xCubic.derivative(time))) + headingOffsetDegrees);
	}
	@Override
	public double calculateDistance(double tStart, double tEnd) {
		tStart = putInRange(tStart);
		tEnd = putInRange(tEnd);
		double distance = 0;
		for (double t = tStart; t < tEnd; t += sampleRate) {
			distance += Pos2D.getDistanceBetweenCoordinates(getPosition(t),getPosition(t+sampleRate));
		}
		return distance;
	}
	@Override
	public double getSpeedRate(){
		return speed;
	}
	private double putInRange(double timeUnit) {
		timeUnit = Math.min(timeUnit, 1);
		timeUnit = Math.max(timeUnit, 0);
		return timeUnit;
	}
	@Override
	public SimpleMatrix getSplineFormula(){
		return xCubic.getCoeffs().combine(1,0, yCubic.getCoeffs());
	}
}
