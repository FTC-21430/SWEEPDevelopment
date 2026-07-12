package org.firstinspires.ftc.teamcode.SWEEP.Splines;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;

/**
 * Represents a time-bounded motion segment that can report pose and velocity over absolute time.
 */
public interface Segment {

	/**
	 * Computes segment position at an absolute timestamp.
	 *
	 * @param overallTime absolute time in seconds
	 * @return pose at the requested time
	 */
	Coordinate getPosition(double overallTime);
	/**
	 * Calculates the distance traveled between two absolute timestamps.
	 *
	 * @param tStart start time in seconds
	 * @param tEnd end time in seconds
	 * @return distance traveled between the two timestamps
	 */
	default double calculateDistance(double tStart, double tEnd) {
		Coordinate start = getPosition(tStart);
		Coordinate end = getPosition(tEnd);
		return Coordinate.getDistanceBetweenCoordinates(start, end);
	}
}
