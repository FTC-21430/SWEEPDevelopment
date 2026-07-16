package org.firstinspires.ftc.teamcode.SWEEP.Splines.Segments;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;

/**
 * Segment implementation that holds a fixed pose for a configured time window.
 */
public class WaitSegment implements Segment {
	private final double duration;

	/**
	 * Held pose for the entire wait segment.
	 */
	private final Coordinate position;

	/**
	 * Creates a wait segment.
	 *
	 * @param position held pose for this segment
	 * @param duration wait duration in seconds
	 */
	public WaitSegment(Coordinate position, double duration) {
		if (position == null) throw new IllegalArgumentException("position cannot be null");

		this.position = position;
		this.duration = duration;
	}

	/**
	 * @param overallTime absolute time in seconds
	 * @return the held pose for this segment
	 */
	@Override
	public Coordinate getPosition(double overallTime) {
		return position;
	}
	public double getDuration(){
		return duration;
	}
	@Override
	public double getSpeedRate(){
		return 0;
	}
}
