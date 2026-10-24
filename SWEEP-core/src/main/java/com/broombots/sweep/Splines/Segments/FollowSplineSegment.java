package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

/**
 * Follows a Catmull-Rom curve through four control points, from the second point to the third.
 * Heading is the direction of travel from the curve derivative via {@code atan2}, in degrees, plus an optional offset.
 */
public class FollowSplineSegment implements Segment {
	/** Cubic x-position curve, in inches over normalized segment parameter. */
	private final CatmullRomCubic xCubic;
	/** Cubic y-position curve, in inches over normalized segment parameter. */
	private final CatmullRomCubic yCubic;
	/** Increment used to numerically approximate curve distance, as a fraction of the segment. */
	private final double sampleRate = 0.001;
	/** Speed fraction for this segment, taken from its endpoint waypoint. */
	private final double speed;

	/** Constant angle added to the direction of travel to get the robot heading, in degrees. */
	private final double headingOffsetDegrees;

	/**
	 * Creates a following spline segment whose heading is exactly the direction of travel.
	 *
	 * @param p1 control point before the segment start, with position in inches
	 * @param p2 start control point, with position in inches
	 * @param p3 end control point, with position in inches; its speed sets the segment speed
	 * @param p4 control point after the segment end, with position in inches
	 */
	public FollowSplineSegment(Waypoint p1, Waypoint p2, Waypoint p3, Waypoint p4) {
		this(p1, p2, p3, p4, 0);
	}

	/**
	 * Creates a following spline segment whose heading is the direction of travel plus a constant offset.
	 * An offset of 180 drives the robot backwards along the curve.
	 *
	 * @param p1 control point before the segment start, with position in inches
	 * @param p2 start control point, with position in inches
	 * @param p3 end control point, with position in inches; its speed sets the segment speed
	 * @param p4 control point after the segment end, with position in inches
	 * @param headingOffsetDegrees offset added to the direction of travel, in degrees, positive counterclockwise
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
	 * Evaluates the curve and computes heading from the direction of travel.
	 *
	 * @param time normalized curve parameter in the range {@code [0, 1]}
	 * @return pose at that parameter, with position in inches and heading in degrees
	 */
	@Override
	public Pos2D getPosition(double time) {
		return new Pos2D(xCubic.evaluate(time), yCubic.evaluate(time), Math.toDegrees(Math.atan2(yCubic.derivative(time), xCubic.derivative(time))) + headingOffsetDegrees);
	}
	/**
	 * Numerically estimates curve length between normalized segment parameters.
	 *
	 * @param tStart start parameter in the range {@code [0, 1]}
	 * @param tEnd end parameter in the range {@code [0, 1]}
	 * @return estimated distance in inches
	 */
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
	/**
	 * Gets this segment speed fraction relative to the robot maximum speed.
	 *
	 * @return speed fraction in the range {@code [0, 1]}
	 */
	@Override
	public double getSpeedRate(){
		return speed;
	}
	/** Clamps a segment parameter to the supported curve domain. */
	private double putInRange(double timeUnit) {
		timeUnit = Math.min(timeUnit, 1);
		timeUnit = Math.max(timeUnit, 0);
		return timeUnit;
	}
	/**
	 * Gets the cubic x and y coefficient rows for this curve.
	 *
	 * @return a 2x4 matrix, with x coefficients in row 0 and y coefficients in row 1
	 */
	@Override
	public SimpleMatrix getSplineFormula(){
		return xCubic.getCoeffs().combine(1,0, yCubic.getCoeffs());
	}
}
