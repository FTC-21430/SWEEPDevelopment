package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Exercises unusual but plausible waypoint combinations, including waits, duplicate points, and reversals.
 * Robust handling keeps valid user routes usable for path following.
 */
public class SequenceWeirdRouteTest {
	/** Position and route-end comparison tolerance, in inches. */
	private static final double EPS = 1e-3;
	/** Shared movement limits used by generated routes. */
	private static final RobotMovementParameters PARAMS = new DefaultRobotMovementParameters();

	/** @return builder configured with shared test movement limits and sample rate */
	private static SequenceBuilder builder() {
		return new SequenceBuilder(PARAMS).setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE);
	}

	/**
	 * Asserts that a path ends at the supplied coordinate.
	 * @param sequence path to inspect
	 * @param x expected endpoint x coordinate in inches
	 * @param y expected endpoint y coordinate in inches
	 */
	private static void assertEndsAt(Sequence sequence, double x, double y) {
		PathPoint last = sequence.getMovement(sequence.getLastTime());
		assertEquals(x, last.position.x, EPS);
		assertEquals(y, last.position.y, EPS);
	}
	/** Verifies that starting where you end stays put. */
	@Test
	public void startingWhereYouEndStaysPut() {
		Sequence sequence = builder().start(10, 10, 45).end(10, 10, 45).build();

		for (PathPoint point : PathTestSupport.sample(sequence, 0.005)) {
			assertEquals(10, point.position.x, EPS);
			assertEquals(10, point.position.y, EPS);
			assertEquals(0, PathTestSupport.speed(point), 1e-6);
		}
		PathTestSupport.assertFiniteHeading(sequence);
	}
	/** Verifies that a move of one hundredth of an inch still completes. */
	@Test
	public void aMoveOfOneHundredthOfAnInchStillCompletes() {
		Sequence sequence = builder().start(0, 0, 0).end(0.01, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 0.01, 0);
	}
	/** Verifies that waiting at the start holds the robot in place. */
	@Test
	public void waitingAtTheStartHoldsTheRobotInPlace() {
		Sequence sequence = builder().start(0, 0, 0).waitAt(1).splineTo(20, 0, 1).end(40, 0, 0).build();

		for (double t = 0; t < 0.9; t += 0.01) {
			PathPoint point = sequence.getMovement(t);
			assertEquals(0, point.position.x, EPS);
			assertEquals(0, PathTestSupport.speed(point), 1e-6);
		}
		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that break at the start still leaves the start. */
	@Test
	public void breakAtTheStartStillLeavesTheStart() {
		Sequence sequence = builder().start(0, 0, 0).addBreak().splineTo(20, 0, 1).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that back to back waits hold for their combined duration. */
	@Test
	public void backToBackWaitsHoldForTheirCombinedDuration() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).waitAt(1).end(40, 0, 0).build();

		assertEquals(2.0, PathTestSupport.restDurationAt(sequence, new Pos2D(20, 0, 0)), 0.02);
		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that waiting right before the end holds then finishes. */
	@Test
	public void waitingRightBeforeTheEndHoldsThenFinishes() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).end(40, 0, 0).build();

		assertEquals(1.0, PathTestSupport.restDurationAt(sequence, new Pos2D(20, 0, 0)), 0.02);
		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that heading stays finite after asingle wait. */
	@Test
	public void headingStaysFiniteAfterASingleWait() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).splineTo(30, 0, 1)
				.end(40, 0, 0).build();

		PathTestSupport.assertFiniteHeading(sequence);
	}
	/** Verifies that heading stays finite after back to back waits. */
	@Test
	public void headingStaysFiniteAfterBackToBackWaits() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).waitAt(1).end(40, 0, 0).build();

		PathTestSupport.assertFiniteHeading(sequence);
	}
	/** Verifies that zero duration wait is harmless. */
	@Test
	public void zeroDurationWaitIsHarmless() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(0).end(40, 0, 0).build();

		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that negative duration wait is harmless. */
	@Test
	public void negativeDurationWaitIsHarmless() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(-1).end(40, 0, 0).build();

		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that doubling back on itself stays on the line. */
	@Test
	public void doublingBackOnItselfStaysOnTheLine() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(30, 0, 1).splineTo(0, 0, 1).end(-30, 0, 0).build();
		double furthest = 0;

		// the robot slows to a near stop at the turnaround, so it needs fine sampling to be seen there
		for (PathPoint point : PathTestSupport.sample(sequence, 1e-5)) {
			assertEquals(0, point.position.y, EPS);
			furthest = Math.max(furthest, point.position.x);
		}
		assertTrue("Never reached the turnaround, furthest was " + furthest, furthest >= 30 - EPS);
		assertEndsAt(sequence, -30, 0);
		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		PathTestSupport.assertSpeedWithinLimit(sequence, PARAMS);
	}
	/** Verifies that duplicate waypoints do not break the path. */
	@Test
	public void duplicateWaypointsDoNotBreakThePath() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).splineTo(20, 0, 1).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		PathTestSupport.assertTimesNeverDecrease(sequence);
		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that zero speed waypoint still finishes the route. */
	@Test
	public void zeroSpeedWaypointStillFinishesTheRoute() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 0).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 40, 0);
	}

	/** Heading must cross the 360-degree wrap by the short route rather than nearly a full revolution. */
	@Test
	public void headingTakesTheShortWayAcrossTheWrapPoint() {
		Sequence sequence = builder().start(0, 0, 350).splineToAngle(20, 0, 10, 1).end(40, 0, 370).build();
		double lowest = Double.MAX_VALUE;
		double highest = -Double.MAX_VALUE;

		for (PathPoint point : PathTestSupport.sample(sequence, 0.005)) {
			lowest = Math.min(lowest, point.position.angle);
			highest = Math.max(highest, point.position.angle);
		}
		assertTrue("Heading swung through " + (highest - lowest) + " degrees going from 350 to 10",
				highest - lowest <= 90);
	}
	/** Verifies that end heading matches the requested heading. */
	@Test
	public void endHeadingMatchesTheRequestedHeading() {
		for (double angle : new double[]{0, 90, -90, 180}) {
			Sequence sequence = builder().start(0, 0, 0).splineTo(20, 10, 1).end(40, 0, angle).build();

			assertEquals(angle, sequence.getMovement(sequence.getLastTime()).position.angle, EPS);
		}
	}
	/** Verifies that long zig zag route stays well formed. */
	@Test
	public void longZigZagRouteStaysWellFormed() {
		SequenceBuilder builder = builder().start(-60, -60, 0);
		for (int i = 0; i < 30; i++) {
			builder.splineTo(-60 + i * 4, i % 2 == 0 ? -40 : 40, 1);
		}
		Sequence sequence = builder.end(60, 60, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		PathTestSupport.assertTimesNeverDecrease(sequence);
		PathTestSupport.assertSpeedWithinLimit(sequence, PARAMS);
		assertEndsAt(sequence, 60, 60);
	}
	/** Verifies that coarse sample rate still reaches the end. */
	@Test
	public void coarseSampleRateStillReachesTheEnd() {
		Sequence sequence = new SequenceBuilder(PARAMS).setSampleRate(0.05)
				.start(0, 0, 0).splineTo(20, 10, 1).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 40, 0);
	}
	/** Verifies that path time never runs backwards across waits and breaks. */
	@Test
	public void pathTimeNeverRunsBackwardsAcrossWaitsAndBreaks() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 10, 1).waitAt(0.5).splineTo(40, 0, 1).addBreak()
				.splineTo(20, -20, 1).end(0, 0, 0).build();

		PathTestSupport.assertTimesNeverDecrease(sequence);
	}
}
