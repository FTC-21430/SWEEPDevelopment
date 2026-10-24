package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

/**
 * Segment implementation that holds a fixed pose for a configured time window.
 */
public class WaitSegment implements Segment {
	/** Duration of the pose hold, in seconds. */
	private final double duration;

	/**
	 * Held pose for the entire wait segment, with position in inches and heading in degrees.
	 */
	private final Pos2D position;

	/**
	 * Creates a wait segment.
	 *
	 * @param position held pose for this segment, with position in inches and heading in degrees
	 * @param duration wait duration in seconds
	 * @throws IllegalArgumentException if position is null
	 */
	public WaitSegment(Pos2D position, double duration) {
		if (position == null) throw new IllegalArgumentException("position cannot be null");

		this.position = position;
		this.duration = duration;
	}

	/**
	 * Returns the same held pose for every supplied parameter.
	 *
	 * @param overallTime ignored; the held pose is constant for all parameter values
	 * @return the held pose for this segment, with position in inches and heading in degrees
	 */
	@Override
	public Pos2D getPosition(double overallTime) {
		return position;
	}
	/**
	 * Gets the length of time for which this segment holds its pose.
	 *
	 * @return wait duration in seconds
	 */
	public double getDuration(){
		return duration;
	}
	/**
	 * Gets the wait segment speed fraction.
	 *
	 * @return zero because the robot is stationary during the wait
	 */
	@Override
	public double getSpeedRate(){
		return 0;
	}
	/**
	 * Gets the constant x/y polynomial coefficients for the held pose.
	 *
	 * @return a 2x4 matrix with each row's constant coefficient in column 3
	 */
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
