package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Unusual waypoint sequences that a user could plausibly write. */
public class SequenceWeirdRouteTest {
	private static final double EPS = 1e-3;
	private static final RobotMovementParameters PARAMS = new DefaultRobotMovementParameters();

	private static SequenceBuilder builder() {
		return new SequenceBuilder(PARAMS).setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE);
	}

	private static void assertEndsAt(Sequence sequence, double x, double y) {
		PathPoint last = sequence.getMovement(sequence.getLastTime());
		assertEquals(x, last.position.x, EPS);
		assertEquals(y, last.position.y, EPS);
	}

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

	@Test
	public void aMoveOfOneHundredthOfAnInchStillCompletes() {
		Sequence sequence = builder().start(0, 0, 0).end(0.01, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 0.01, 0);
	}

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

	@Test
	public void breakAtTheStartStillLeavesTheStart() {
		Sequence sequence = builder().start(0, 0, 0).addBreak().splineTo(20, 0, 1).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 40, 0);
	}

	@Test
	public void backToBackWaitsHoldForTheirCombinedDuration() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).waitAt(1).end(40, 0, 0).build();

		assertEquals(2.0, PathTestSupport.restDurationAt(sequence, new Pos2D(20, 0, 0)), 0.02);
		assertEndsAt(sequence, 40, 0);
	}

	@Test
	public void waitingRightBeforeTheEndHoldsThenFinishes() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).end(40, 0, 0).build();

		assertEquals(1.0, PathTestSupport.restDurationAt(sequence, new Pos2D(20, 0, 0)), 0.02);
		assertEndsAt(sequence, 40, 0);
	}

	@Test
	public void headingStaysFiniteAfterASingleWait() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).splineTo(30, 0, 1)
				.end(40, 0, 0).build();

		PathTestSupport.assertFiniteHeading(sequence);
	}

	@Test
	public void headingStaysFiniteAfterBackToBackWaits() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(1).waitAt(1).end(40, 0, 0).build();

		PathTestSupport.assertFiniteHeading(sequence);
	}

	@Test
	public void zeroDurationWaitIsHarmless() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(0).end(40, 0, 0).build();

		assertEndsAt(sequence, 40, 0);
	}

	@Test
	public void negativeDurationWaitIsHarmless() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).waitAt(-1).end(40, 0, 0).build();

		assertEndsAt(sequence, 40, 0);
	}

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

	@Test
	public void duplicateWaypointsDoNotBreakThePath() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 1).splineTo(20, 0, 1).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		PathTestSupport.assertTimesNeverDecrease(sequence);
		assertEndsAt(sequence, 40, 0);
	}

	@Test
	public void zeroSpeedWaypointStillFinishesTheRoute() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 0, 0).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 40, 0);
	}

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

	@Test
	public void endHeadingMatchesTheRequestedHeading() {
		for (double angle : new double[]{0, 90, -90, 180}) {
			Sequence sequence = builder().start(0, 0, 0).splineTo(20, 10, 1).end(40, 0, angle).build();

			assertEquals(angle, sequence.getMovement(sequence.getLastTime()).position.angle, EPS);
		}
	}

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

	@Test
	public void coarseSampleRateStillReachesTheEnd() {
		Sequence sequence = new SequenceBuilder(PARAMS).setSampleRate(0.05)
				.start(0, 0, 0).splineTo(20, 10, 1).end(40, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEndsAt(sequence, 40, 0);
	}

	@Test
	public void pathTimeNeverRunsBackwardsAcrossWaitsAndBreaks() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(20, 10, 1).waitAt(0.5).splineTo(40, 0, 1).addBreak()
				.splineTo(20, -20, 1).end(0, 0, 0).build();

		PathTestSupport.assertTimesNeverDecrease(sequence);
	}
}
