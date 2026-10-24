package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Checks where generated paths stop and how long stationary holds last at waits, breaks, and the endpoint.
 * Predictable stops are necessary for the robot to perform waypoint actions and honor pauses.
 */
public class SequenceStopTest {
	private static final double POSITION_TOLERANCE = 1e-3; // inches
	private static final double AT_REST = 1e-6; // inches/second
	private static final double TIME_STEP = 1e-4; // seconds, the resolution hold durations are measured at
	private static final double END_HOLD = 0.4; // seconds the builder holds at the END waypoint

	/** @return standard route builder starting from rest at the origin */
	private static SequenceBuilder baseBuilder() {
		return new SequenceBuilder(new DefaultRobotMovementParameters())
				.setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE)
				.start(0, 0, 0);
	}

	/**
	 * Measures sampled time spent stationary at the target.
	 * @param sequence path to inspect
	 * @param target target position, in inches
	 * @return stationary duration in seconds
	 */
	private static double restDurationAt(Sequence sequence, Pos2D target) {
		int restingSamples = 0;
		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			boolean atTarget = PathTestSupport.distance(point.position, target) <= POSITION_TOLERANCE;
			if (atTarget && PathTestSupport.speed(point) <= AT_REST) restingSamples++;
		}
		return restingSamples * TIME_STEP;
	}
	/** Verifies that robot starts at rest. */
	@Test
	public void robotStartsAtRest() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).end(48, 0, 0).build();

		assertEquals(0, PathTestSupport.speed(sequence.getMovement(0)), AT_REST);
	}
	/** Verifies that robot comes to rest at the end waypoint. */
	@Test
	public void robotComesToRestAtTheEndWaypoint() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).end(48, 0, 90).build();

		PathPoint last = sequence.getMovement(sequence.getLastTime());
		assertEquals(0, PathTestSupport.speed(last), AT_REST);
		assertEquals(0, PathTestSupport.distance(last.position, new Pos2D(48, 0, 0)), POSITION_TOLERANCE);
	}
	/** Verifies that robot holds at the end waypoint for the final hold. */
	@Test
	public void robotHoldsAtTheEndWaypointForTheFinalHold() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).end(48, 0, 0).build();

		assertEquals(END_HOLD, restDurationAt(sequence, new Pos2D(48, 0, 0)), 0.01);
	}
	/** Verifies that robot never leaves the end waypoint once it arrives. */
	@Test
	public void robotNeverLeavesTheEndWaypointOnceItArrives() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).end(48, 0, 0).build();
		Pos2D end = new Pos2D(48, 0, 0);
		double arrival = Double.NaN;

		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			boolean atEnd = PathTestSupport.distance(point.position, end) <= POSITION_TOLERANCE;
			if (Double.isNaN(arrival) && atEnd) arrival = point.time;
			if (!Double.isNaN(arrival)) assertTrue(atEnd);
		}
		assertTrue("Robot never reached the end", !Double.isNaN(arrival));
	}

	/** The 0.01-second tolerance covers quantization from the 1e-4-second sampling interval. */
	@Test
	public void waitAtStopsAtTheWaypointForTheRequestedDuration() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).waitAt(2.5).splineTo(48, 12, 1).end(72, 0, 0).build();

		assertEquals(2.5, restDurationAt(sequence, new Pos2D(24, 12, 0)), 0.01);
	}
	/** Verifies that different wait durations produce proportional holds. */
	@Test
	public void differentWaitDurationsProduceProportionalHolds() {
		for (double duration : new double[]{0.5, 1.0, 3.0}) {
			Sequence sequence = baseBuilder().splineTo(24, 12, 1).waitAt(duration).splineTo(48, 12, 1)
					.end(72, 0, 0).build();

			assertEquals(duration, restDurationAt(sequence, new Pos2D(24, 12, 0)), 0.01);
		}
	}
	/** Verifies that waiting adds its duration to the total path time. */
	@Test
	public void waitingAddsItsDurationToTheTotalPathTime() {
		Sequence withoutWait = baseBuilder().splineTo(24, 12, 1).splineTo(48, 12, 1).end(72, 0, 0).build();
		Sequence withWait = baseBuilder().splineTo(24, 12, 1).waitAt(2.0).splineTo(48, 12, 1).end(72, 0, 0).build();

		assertTrue(withWait.getLastTime() > withoutWait.getLastTime() + 2.0);
	}
	/** Verifies that break waypoint stops briefly at the waypoint. */
	@Test
	public void breakWaypointStopsBrieflyAtTheWaypoint() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).addBreak().splineTo(48, 12, 1).end(72, 0, 0).build();

		assertEquals(1e-8, restDurationAt(sequence, new Pos2D(24, 12, 0)), 0.01);
	}
	/** Verifies that ordinary spline waypoint is passed without stopping. */
	@Test
	public void ordinarySplineWaypointIsPassedWithoutStopping() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).splineTo(48, 12, 1).end(72, 0, 0).build();
		Pos2D waypoint = new Pos2D(24, 12, 0);
		double passingSpeed = 0;
		double closest = Double.MAX_VALUE;

		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			double distance = PathTestSupport.distance(point.position, waypoint);
			if (distance < closest) {
				closest = distance;
				passingSpeed = PathTestSupport.speed(point);
			}
		}
		assertTrue("Robot slowed to " + passingSpeed + " in/s at a plain spline waypoint", passingSpeed > 5);
	}
	/** Verifies that robot is stopped for the whole wait and moves before and after. */
	@Test
	public void robotIsStoppedForTheWholeWaitAndMovesBeforeAndAfter() {
		Sequence sequence = baseBuilder().splineTo(24, 12, 1).waitAt(1.0).splineTo(48, 12, 1).end(72, 0, 0).build();
		ArrayList<PathPoint> samples = PathTestSupport.sample(sequence, TIME_STEP);
		Pos2D waypoint = new Pos2D(24, 12, 0);

		double firstRest = Double.NaN;
		double lastRest = Double.NaN;
		for (PathPoint point : samples) {
			boolean resting = PathTestSupport.distance(point.position, waypoint) <= POSITION_TOLERANCE
					&& PathTestSupport.speed(point) <= AT_REST;
			if (!resting) continue;
			if (Double.isNaN(firstRest)) firstRest = point.time;
			lastRest = point.time;
		}
		assertTrue(firstRest > 0.1);
		assertTrue(lastRest < sequence.getLastTime() - END_HOLD);
		assertEquals(1.0, lastRest - firstRest, 0.01);
	}
}
