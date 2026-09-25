package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;

import org.ejml.simple.SimpleMatrix;

/**
 * Represents a time-bounded motion segment that can report pose and velocity over absolute time.
 */
public interface Segment {

	/**
	 * Computes segment position at an absolute timestamp.
	 *
	 * @param t absolute time in seconds
	 * @return pose at the requested time
	 */
	Pos2D getPosition(double t);
	/**
	 * Calculates the distance traveled between two absolute timestamps.
	 *
	 * @param tStart start time in seconds
	 * @param tEnd end time in seconds
	 * @return distance traveled between the two timestamps
	 */
	default double calculateDistance(double tStart, double tEnd) {
		Pos2D start = getPosition(tStart);
		Pos2D end = getPosition(tEnd);
		return Pos2D.getDistanceBetweenCoordinates(start,end);
	}
	double getSpeedRate();
	/**
	 * ax, bx, cx, dx,
	 * ay, by, cy, dy
	 */
	SimpleMatrix getSplineFormula();

}
