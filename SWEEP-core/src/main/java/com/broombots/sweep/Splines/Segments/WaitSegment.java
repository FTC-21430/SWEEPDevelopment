package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

/**
 * Segment implementation that holds a fixed pose for a configured time window.
 */
public class WaitSegment implements Segment {
	private final double duration;

	/**
	 * Held pose for the entire wait segment.
	 */
	private final Pos2D position;

	/**
	 * Creates a wait segment.
	 *
	 * @param position held pose for this segment
	 * @param duration wait duration in seconds
	 */
	public WaitSegment(Pos2D position, double duration) {
		if (position == null) throw new IllegalArgumentException("position cannot be null");

		this.position = position;
		this.duration = duration;
	}

	/**
	 * @param overallTime absolute time in seconds
	 * @return the held pose for this segment
	 */
	@Override
	public Pos2D getPosition(double overallTime) {
		return position;
	}
	public double getDuration(){
		return duration;
	}
	@Override
	public double getSpeedRate(){
		return 0;
	}
	@Override
	public SimpleMatrix getSplineFormula(){
		return new SimpleMatrix(new double[][]{
				{
					0, 0, 0, position.x
				},
				{
					0 ,0, 0, position.y
				}
		});
	}
}
