package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Checks that a replacement path starts from the current pose and velocity of a path already in progress.
 * Preserving that handoff helps avoid a discontinuity in motion during path following.
 */
public class SequenceMidPathSwitchTest {
	/** Position, velocity, and timing comparison tolerance; units follow each asserted value. */
	private static final double EPS = 1e-3;
	/** Shared movement limits for both the original and replacement routes. */
	private static final RobotMovementParameters PARAMS = new DefaultRobotMovementParameters();

	/** @return builder configured with shared movement limits and path sampling step */
	private static SequenceBuilder builder() {
		return new SequenceBuilder(PARAMS).setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE);
	}

	/** @return test route along the x axis, with positions in inches and headings in degrees */
	private static Sequence straightRun() {
		return builder().start(0, 0, 0).splineTo(30, 0, 1).end(60, 0, 0).build();
	}

	/**
	 * Finds an approximate time of peak speed so the handoff has clear nonzero velocity.
	 * @param sequence straight route to inspect
	 * @return approximate peak-speed time in seconds
	 */
	private static double fastestTime(Sequence sequence) {
		double bestTime = 0;
		double bestSpeed = -1;
		for (double t = 0; t < sequence.getLastTime(); t += 0.01) {
			double speed = PathTestSupport.speed(sequence.getMovement(t));
			if (speed > bestSpeed) {
				bestSpeed = speed;
				bestTime = t;
			}
		}
		return bestTime;
	}

	/**
	 * Creates a replacement route from the old route's pose and velocity at handoff.
	 * @param running route being replaced
	 * @param time handoff time in seconds
	 * @param endX new endpoint x coordinate in inches
	 * @param endY new endpoint y coordinate in inches
	 * @return replacement sequence
	 */
	private static Sequence switchAt(Sequence running, double time, double endX, double endY) {
		PathPoint handoff = running.getMovement(time);
		return builder().start(handoff).end(endX, endY, 0).build();
	}

	/** A speed above 10 inches per second confirms the selected handoff is meaningfully in motion. */
	@Test
	public void handoffMoment_hasRealVelocity() {
		Sequence running = straightRun();

		assertTrue(PathTestSupport.speed(running.getMovement(fastestTime(running))) > 10);
	}
	/** Verifies that new path starts where the old one was. */
	@Test
	public void newPathStartsWhereTheOldOneWas() {
		Sequence running = straightRun();
		double time = fastestTime(running);
		Pos2D expected = running.getMovement(time).position;

		Sequence switched = switchAt(running, time, 40, 30);

		Pos2D start = switched.getMovement(0).position;
		assertEquals(expected.x, start.x, EPS);
		assertEquals(expected.y, start.y, EPS);
	}
	/** Verifies that new path runs on its own clock from zero. */
	@Test
	public void newPathRunsOnItsOwnClockFromZero() {
		Sequence running = straightRun();
		double time = fastestTime(running);

		Sequence switched = switchAt(running, time, 40, 30);

		// start(PathPoint) resets the start time, so the interpreter can restart its timer for the new path
		assertEquals(0, switched.getMovement(0).time, EPS);
		assertTrue(switched.getLastTime() > 0);
	}
	/** Verifies that new path starts with the velocity the robot had. */
	@Test
	public void newPathStartsWithTheVelocityTheRobotHad() {
		Sequence running = straightRun();
		double time = fastestTime(running);
		Pos2D velocity = running.getMovement(time).velocity;

		Sequence switched = switchAt(running, time, 40, 30);

		Pos2D start = switched.getMovement(0).velocity;
		assertEquals(velocity.x, start.x, 0.5);
		assertEquals(velocity.y, start.y, 0.5);
	}

	/** The 0.3-second probe checks how inherited forward velocity shapes the initial curve. */
	@Test
	public void newPathKeepsMovingInTheOldDirectionBeforeTurning() {
		Sequence running = straightRun();
		double time = fastestTime(running);
		Pos2D from = running.getMovement(time).position;

		// A stationary start would head straight at the target; the moving start must carry on in +x first.
		Sequence moving = switchAt(running, time, from.x, from.y + 30);
		PathPoint stationary = new PathPoint();
		stationary.time = time;
		stationary.position = new Pos2D(from.x, from.y, 0);
		stationary.velocity = new Pos2D(0, 0, 0);
		Sequence fromRest = builder().start(stationary).end(from.x, from.y + 30, 0).build();

		double probe = 0.3;
		assertTrue("Moving handoff should have travelled further in x than one from rest",
				moving.getMovement(probe).position.x > fromRest.getMovement(probe).position.x + 0.1);
	}
	/** Verifies that target straight ahead is not overshot by much more than braking requires. */
	@Test
	public void targetStraightAheadIsNotOvershotByMuchMoreThanBrakingRequires() {
		Sequence running = straightRun();
		double time = fastestTime(running);
		double targetX = running.getMovement(time).position.x + 40;

		Sequence switched = switchAt(running, time, targetX, 0);

		double furthest = -Double.MAX_VALUE;
		for (PathPoint point : PathTestSupport.sample(switched, 0.01)) {
			furthest = Math.max(furthest, point.position.x);
		}
		assertTrue("Path ran to x=" + furthest + " for a target at x=" + targetX, furthest <= targetX + 5);
	}
	/** Verifies that switched path stays finite and reaches its new target. */
	@Test
	public void switchedPathStaysFiniteAndReachesItsNewTarget() {
		Sequence running = straightRun();
		double time = fastestTime(running);

		Sequence switched = switchAt(running, time, 40, 30);

		PathTestSupport.assertFinitePoseAndVelocity(switched);
		PathPoint end = switched.getMovement(switched.getLastTime());
		assertEquals(40, end.position.x, EPS);
		assertEquals(30, end.position.y, EPS);
	}
	/** Verifies that switching does not change the original path. */
	@Test
	public void switchingDoesNotChangeTheOriginalPath() {
		Sequence running = straightRun();
		double time = fastestTime(running);
		Pos2D before = running.getMovement(time).position;
		double lastBefore = running.getLastTime();

		switchAt(running, time, 40, 30);

		assertEquals(lastBefore, running.getLastTime(), EPS);
		assertEquals(before.x, running.getMovement(time).position.x, EPS);
	}
	/** Verifies that switching after the run has stopped still builds. */
	@Test
	public void switchingAfterTheRunHasStoppedStillBuilds() {
		Sequence running = straightRun();
		double endTime = running.getLastTime();

		Sequence switched = switchAt(running, endTime, 80, 0);

		PathTestSupport.assertFinitePoseAndVelocity(switched);
		assertEquals(80, switched.getMovement(switched.getLastTime()).position.x, EPS);
	}

	/**
	 * Creates a start point moving along positive x.
	 * @param speed initial speed in inches per second
	 * @return moving start point at the origin
	 */
	private static PathPoint movingStart(double speed) {
		PathPoint point = new PathPoint();
		point.position = new Pos2D(0, 0, 0);
		point.velocity = new Pos2D(speed, 0, 0);
		return point;
	}

	/**
	 * Computes ideal stopping distance under the configured translational acceleration.
	 * @param speed initial speed in inches per second
	 * @return braking distance in inches
	 */
	private static double brakingDistance(double speed) {
		return speed * speed / (2 * PARAMS.getMaxStableAcceleration(0));
	}

	/**
	 * Finds the furthest x coordinate reached by sampled points.
	 * @param sequence path to inspect
	 * @return maximum x coordinate in inches
	 */
	private static double furthestX(Sequence sequence) {
		double furthest = -Double.MAX_VALUE;
		for (PathPoint point : PathTestSupport.sample(sequence, 0.01)) {
			furthest = Math.max(furthest, point.position.x);
		}
		return furthest;
	}
	/** Verifies that starting at rest ignores the velocity handle. */
	@Test
	public void startingAtRestIgnoresTheVelocityHandle() {
		Sequence fresh = builder().start(0, 0, 0).end(40, 0, 0).build();
		Sequence fromRest = builder().start(movingStart(0)).end(40, 0, 0).build();

		assertEquals(fresh.getLastTime(), fromRest.getLastTime(), EPS);
	}
	/** Verifies that target beyond the braking distance is reached without overshoot. */
	@Test
	public void targetBeyondTheBrakingDistanceIsReachedWithoutOvershoot() {
		double speed = 40;
		double target = brakingDistance(speed) + 40;

		Sequence switched = builder().start(movingStart(speed)).end(target, 0, 0).build();

		assertTrue("Ran to x=" + furthestX(switched) + " for target " + target, furthestX(switched) <= target + 1);
		assertEquals(target, switched.getMovement(switched.getLastTime()).position.x, EPS);
	}
	/** Verifies that target closer than the braking distance still builds and ends on target. */
	@Test
	public void targetCloserThanTheBrakingDistanceStillBuildsAndEndsOnTarget() {
		// Accepted limitation: the speed profile does not carry the start speed forward, so for a target this close it
		// asks for more braking than the robot can do. The path itself must still be valid and end on the target.
		double speed = 60;
		double target = brakingDistance(speed) / 2;

		Sequence switched = builder().start(movingStart(speed)).end(target, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(switched);
		assertEquals(target, switched.getMovement(switched.getLastTime()).position.x, EPS);
	}
	/** Verifies that target behind the robot is still reached. */
	@Test
	public void targetBehindTheRobotIsStillReached() {
		Sequence switched = builder().start(movingStart(40)).end(-30, 0, 0).build();

		PathTestSupport.assertFinitePoseAndVelocity(switched);
		assertEquals(-30, switched.getMovement(switched.getLastTime()).position.x, EPS);
	}
	/** Verifies that faster start carries further along its heading before turning. */
	@Test
	public void fasterStartCarriesFurtherAlongItsHeadingBeforeTurning() {
		double slowY = builder().start(movingStart(10)).end(0, 60, 0).build().getMovement(0.5).position.x;
		double fastX = builder().start(movingStart(50)).end(0, 60, 0).build().getMovement(0.5).position.x;

		assertTrue("Fast start x=" + fastX + " should exceed slow start x=" + slowY, fastX > slowY);
	}
	/** Verifies that starting velocity direction is respected for sideways motion. */
	@Test
	public void startingVelocityDirectionIsRespectedForSidewaysMotion() {
		PathPoint sideways = new PathPoint();
		sideways.position = new Pos2D(0, 0, 0);
		sideways.velocity = new Pos2D(0, 30, 0);

		Sequence switched = builder().start(sideways).end(40, 0, 0).build();

		double bulge = 0;
		for (PathPoint point : PathTestSupport.sample(switched, 0.01)) {
			bulge = Math.max(bulge, point.position.y);
		}
		// a path started from rest along the x axis never leaves y = 0, so any +y bulge comes from the start velocity
		assertTrue("Should bulge towards +y, got " + bulge, bulge > 0.3);
		PathTestSupport.assertFinitePoseAndVelocity(switched);
	}
}
