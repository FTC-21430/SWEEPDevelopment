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
	 * @param overallTime absolute time in seconds
	 * @return the held pose for this segment
	 */
	@Override
	public Coordinate getPosition(double overallTime) {
		return position;
	}
}
