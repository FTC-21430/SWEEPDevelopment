package org.firstinspires.ftc.teamcode.SWEEP.Splines.Segments;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.CatmullRomCubic;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.SplineWaypoint;

/**
 * Placeholder segment for direct spline-follow logic that is not yet implemented.
 */
public class FollowSplineSegment implements Segment {
	CatmullRomCubic xCubic;
	CatmullRomCubic yCubic;
	private final double sampleRate = 0.001; // units

	/**
	 * Creates a following spline segment
	 */
	public FollowSplineSegment(Waypoint p1, Waypoint p2, SplineWaypoint p3, Waypoint p4) {
		xCubic = new CatmullRomCubic(p1.getX(), p2.getX(), p3.getX(), p4.getX());
		yCubic = new CatmullRomCubic(p1.getY(), p2.getY(), p3.getY(), p4.getY());
	}
	/**
	 * @param time absolute time ( must range between 0-1 )
	 * @return pose at the requested time
	 */
	@Override
	public Coordinate getPosition(double time) {
		return new Coordinate(xCubic.evaluate(time), yCubic.evaluate(time), Math.toDegrees(Math.atan2(yCubic.derivative(time), xCubic.derivative(time))));
	}
	@Override
	public double calculateDistance(double tStart, double tEnd) {
		tStart = putInRange(tStart);
		tEnd = putInRange(tEnd);
		double distance = 0;
		for (double t = tStart; t < tEnd; t += sampleRate) {
			distance += Coordinate.getDistanceBetweenCoordinates(getPosition(t),getPosition(t+sampleRate));
		}
		return distance;
	}
	private double putInRange(double timeUnit) {
		timeUnit = Math.min(timeUnit, 1);
		timeUnit = Math.max(timeUnit, 0);
		return timeUnit;
	}

}
