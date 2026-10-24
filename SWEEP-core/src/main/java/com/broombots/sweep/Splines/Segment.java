package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;

import org.ejml.simple.SimpleMatrix;

/**
 * Represents a path segment that reports pose and speed information along its curve.
 */
public interface Segment {

	/**
	 * Computes the pose at a normalized parameter along this segment.
	 *
	 * @param t normalized parameter in the range {@code [0, 1]}; movement segments evaluate the curve at this value
	 * @return pose at the requested point on the segment, with position in inches and heading in degrees
	 */
	Pos2D getPosition(double t);
	/**
	 * Calculates the distance traveled between two normalized parameters on this segment.
	 *
	 * @param tStart start parameter in the range {@code [0, 1]}
	 * @param tEnd end parameter in the range {@code [0, 1]}
	 * @return distance traveled in inches between the two parameters
	 */
	default double calculateDistance(double tStart, double tEnd) {
		Pos2D start = getPosition(tStart);
		Pos2D end = getPosition(tEnd);
		return Pos2D.getDistanceBetweenCoordinates(start,end);
	}
	/**
	 * Gets the segment speed as a fraction of the robot maximum speed.
	 *
	 * @return speed fraction in the range {@code [0, 1]}
	 */
	double getSpeedRate();
	@Deprecated
	/**
	 * Gets the cubic coefficients for the segment x and y curves. Row 0 contains x coefficients {@code [a, b, c, d]},
	 * and row 1 contains y coefficients {@code [a, b, c, d]}, where each curve is
	 * {@code f(t) = a*t^3 + b*t^2 + c*t + d}.
	 *
	 * @return a 2x4 matrix of cubic coefficients
	 */
	SimpleMatrix getSplineFormula();

}
