package org.firstinspires.ftc.teamcode.SWEEP.Splines.Segments;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;

/**
 * Segment implementation that holds a fixed pose for a configured time window.
 */
public class WaitSegment implements Segment {
	/**
	 * Absolute start time for this segment in seconds.
	 */
	private final double startTime;

	/**
	 * Absolute end time for this segment in seconds.
	 */
	private final double endTime;

	/**
	 * Held pose for the entire wait segment.
	 */
	private final Coordinate position;

	/**
	 * Creates a wait segment.
	 *
	 * @param position held pose for this segment
	 * @param startTime absolute start time in seconds
	 * @param duration wait duration in seconds
	 */
	public WaitSegment(Coordinate position, double startTime, double duration) {
		if (position == null) throw new IllegalArgumentException("position cannot be null");

		this.position = position;
		this.startTime = startTime;
		this.endTime = startTime + duration;
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
	 * @param time absolute time in seconds
	 * @return true when {@code time} is within [startTime, endTime)
	 */
	@Override
	public boolean activeAt(double time) {
		return startTime <= time && time < endTime;
	}

	/**
	 * @param overallTime absolute time in seconds
	 * @return the held pose for this segment
	 */
	@Override
	public Coordinate getPosition(double overallTime) {
		return position;
	}

	/**
	 * @param overallTime absolute time in seconds
	 * @return [0, 0] because wait segments do not translate
	 */
	@Override
	public double[] getVelocity(double overallTime) {
		return new double[]{0, 0};
	}
}
