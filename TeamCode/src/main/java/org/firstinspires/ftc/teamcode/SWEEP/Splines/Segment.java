package org.firstinspires.ftc.teamcode.SWEEP.Splines;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;

/**
 * Represents a time-bounded motion segment that can report pose and velocity over absolute time.
 */
public interface Segment {
	/**
	 * @return absolute start time for this segment in seconds
	 */
	double getStartTime();

	/**
	 * @return absolute end time for this segment in seconds
	 */
	double getEndTime();

	/**
	 * @return segment duration in seconds
	 */
	default double getTotalTime() {
		return getEndTime() - getStartTime();
	}

	/**
	 * Checks whether this segment should be active at the provided absolute time.
	 *
	 * @param time absolute time in seconds
	 * @return true when {@code time} is within [startTime, endTime)
	 */
	default boolean activeAt(double time) {
		return time >= getStartTime() && time < getEndTime();
	}

	/**
	 * Computes segment position at an absolute timestamp.
	 *
	 * @param overallTime absolute time in seconds
	 * @return pose at the requested time
	 */
	Coordinate getPosition(double overallTime);

	/**
	 * Computes segment position using time relative to this segment start.
	 *
	 * @param independentTime time since segment start in seconds
	 * @return pose at the requested relative time
	 */
	default Coordinate getPositionIndependentTime(double independentTime) {
		return getPosition(getStartTime() + independentTime);
	}

	/**
	 * Computes segment velocity vector at an absolute timestamp.
	 *
	 * @param overallTime absolute time in seconds
	 * @return velocity vector [xDot, yDot]
	 */
	double[] getVelocity(double overallTime);

	/**
	 * Computes segment velocity using time relative to this segment start.
	 *
	 * @param independentTime time since segment start in seconds
	 * @return velocity vector [xDot, yDot]
	 */
	default double[] getVelocityIndependentTime(double independentTime) {
		return getVelocity(getStartTime() + independentTime);
	}

	/**
	 * Gets distance traveled along this segment between two absolute times.
	 * Default behavior uses straight-line distance between sampled endpoints and should
	 * be overridden by curved segments that require arc-length integration.
	 *
	 * @param tStart start absolute time in seconds
	 * @param tEnd end absolute time in seconds
	 * @return distance traveled in inches
	 */
	default double calculateDistance(double tStart, double tEnd) {
		Coordinate start = getPosition(tStart);
		Coordinate end = getPosition(tEnd);
		return Coordinate.getDistanceBetweenCoordinates(start, end);
	}
}
