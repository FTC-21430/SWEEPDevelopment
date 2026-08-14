package org.firstinspires.ftc.teamcode.SWEEP.Splines.Segments;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;

/**
 * Placeholder segment for direct spline-follow logic that is not yet implemented.
 */
public class FollowSplineSegment implements Segment {
	/**
	 * Absolute start time for this segment in seconds.
	 */
	private final double startTime;

	/**
	 * Absolute end time for this segment in seconds.
	 */
	private final double endTime;

	/**
	 * Creates a placeholder follow-spline segment.
	 *
	 * @param startTime absolute start time in seconds
	 */
	public FollowSplineSegment(double startTime) {
		this.startTime = startTime;
		this.endTime = startTime;
	}

	/**
	 * @return absolute start time in seconds
	 */
	@Override
	public double getStartTime() {
		return startTime;
	}

	/**
	 * @return absolute end time in seconds
	 */
	@Override
	public double getEndTime() {
		return endTime;
	}

	/**
	 * @param overallTime absolute time in seconds
	 * @return null until follow-spline runtime behavior is implemented
	 */
	@Override
	public Coordinate getPosition(double overallTime) {
		// TODO: Implement pose lookup for runtime spline-follow segment.
		return null;
	}

	/**
	 * @param overallTime absolute time in seconds
	 * @return empty vector until follow-spline runtime behavior is implemented
	 */
	@Override
	public double[] getVelocity(double overallTime) {
		// TODO: Implement velocity lookup for runtime spline-follow segment.
		return new double[0];
	}
}
