package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Robot;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.SWEEPAction;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Verifies that appending a builder extends a planned path while preserving existing motion and actions.
 * Correct joining lets a running route continue without discontinuities or lost work.
 */
public class SequenceAddToSequenceTest {
	/** Position comparison tolerance, in inches; timing assertions use seconds. */
	private static final double EPS = 1e-3;
	/** Shared movement limits for both joined path segments. */
	private static final RobotMovementParameters PARAMS = new DefaultRobotMovementParameters();

	/** Minimal robot instance for action tests without hardware dependencies. */
	private static class TestRobot extends Robot {
	}

	/** Action that counts executions to verify appended triggers are retained and shifted correctly. */
	private static class FlagAction extends SWEEPAction {
		int executeCount;

		/** @param position action trigger position in inches and heading in degrees */
		FlagAction(Pos2D position) {
			super(new TestRobot(), position, 0, 1.0, 0.0);
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
			executeCount++;
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
		}
	}

	/** @return builder configured with shared test limits and sampling step */
	private static SequenceBuilder builder() {
		return new SequenceBuilder(PARAMS).setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE);
	}

	/** @return first segment of the test route, in inches and degrees */
	private static Sequence firstLeg() {
		return builder().start(0, 0, 0).splineTo(20, 0, 1).end(40, 0, 0).build();
	}

	/** @return builder for the continuation segment, in inches and degrees */
	private static SequenceBuilder secondLeg() {
		return builder().start(40, 0, 0).splineTo(60, 0, 1).end(80, 0, 0);
	}

	/**
	 * Creates localization data at a zero-heading position.
	 * @param x robot x coordinate in inches
	 * @param y robot y coordinate in inches
	 * @return localization packet with zero heading and velocity
	 */
	private static LocalizationPacket packetAt(double x, double y) {
		return new LocalizationPacket(x, y, 0, 0, 0);
	}

	/** The 0.05-second allowance covers the difference between independently sampled segment endpoints. */
	@Test
	public void appendedSequenceExtendsTheTotalDuration() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();
		double secondDuration = secondLeg().build().getLastTime();

		sequence.addToSequence(secondLeg());

		assertEquals(firstDuration + secondDuration, sequence.getLastTime(), 0.05);
	}
	/** Verifies that appended sequence ends at the new endpoint. */
	@Test
	public void appendedSequenceEndsAtTheNewEndpoint() {
		Sequence sequence = firstLeg();

		sequence.addToSequence(secondLeg());

		PathPoint last = sequence.getMovement(sequence.getLastTime());
		assertEquals(80, last.position.x, EPS);
		assertEquals(0, last.position.y, EPS);
	}
	/** Verifies that original portion is left untouched. */
	@Test
	public void originalPortionIsLeftUntouched() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();
		Pos2D before = sequence.getMovement(firstDuration / 2).position;

		sequence.addToSequence(secondLeg());

		Pos2D after = sequence.getMovement(firstDuration / 2).position;
		assertEquals(before.x, after.x, EPS);
		assertEquals(before.y, after.y, EPS);
	}
	/** Verifies that times never decrease across the join. */
	@Test
	public void timesNeverDecreaseAcrossTheJoin() {
		Sequence sequence = firstLeg();

		sequence.addToSequence(secondLeg());

		PathTestSupport.assertTimesNeverDecrease(sequence);
	}
	/** Verifies that path stays finite across the join. */
	@Test
	public void pathStaysFiniteAcrossTheJoin() {
		Sequence sequence = firstLeg();

		sequence.addToSequence(secondLeg());

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		PathTestSupport.assertFiniteHeading(sequence);
	}
	/** Verifies that can append more than once. */
	@Test
	public void canAppendMoreThanOnce() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();

		sequence.addToSequence(secondLeg());
		double afterSecond = sequence.getLastTime();
		sequence.addToSequence(builder().start(80, 0, 0).splineTo(100, 0, 1).end(120, 0, 0));

		assertTrue(afterSecond > firstDuration);
		assertTrue(sequence.getLastTime() > afterSecond);
		assertEquals(120, sequence.getMovement(sequence.getLastTime()).position.x, EPS);
	}
	/** Verifies that appended actions are triggered after their time markers. */
	@Test
	public void appendedActionsAreTriggeredAfterTheirTimeMarkers() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();
		FlagAction action = new FlagAction(new Pos2D(60, 0, 0));
		sequence.addToSequence(secondLeg().addAction(action));

		sequence.updateActions(packetAt(60, 0), 0.0);
		assertEquals("Action fired before its segment started", 0, action.executeCount);

		sequence.updateActions(packetAt(60, 0), sequence.getLastTime());
		assertEquals(1, action.executeCount);
		assertTrue(sequence.getLastTime() > firstDuration);
	}
	/** Verifies that appending actionless sequence keeps existing actions. */
	@Test
	public void appendingActionlessSequenceKeepsExistingActions() {
		FlagAction action = new FlagAction(new Pos2D(0, 0, 0));
		Sequence sequence = builder().start(0, 0, 0).addAction(action).splineTo(20, 0, 1).end(40, 0, 0).build();

		sequence.addToSequence(secondLeg());
		sequence.updateActions(packetAt(0, 0), 0.0);

		assertEquals(1, action.executeCount);
	}
	/** Verifies that appending non zero time sequence to sequence. */
	@Test
	public void appendingNonZeroTimeSequenceToSequence(){
		PathPoint startPoint = new PathPoint();
		startPoint.time = 0;
		startPoint.position = new Pos2D(0,0,0);
		startPoint.velocity = new Pos2D(0,0,0);
		PathPoint chainedPoint = new PathPoint();
		chainedPoint.time = 4;
		chainedPoint.position = new Pos2D(0,40,180);
		chainedPoint.velocity = new Pos2D(0,0,0);
		Sequence originalSequence = builder().start(startPoint).end(chainedPoint.position).build();

		SequenceBuilder newSequence = builder().resetBuilder().start(chainedPoint).end(0,20,180,0.0);
		originalSequence.addToSequence(newSequence);

		Sequence comparisonSequence = builder().resetBuilder().start(startPoint).splineToAngle(chainedPoint.position.x, chainedPoint.position.y, chainedPoint.position.angle, 1.0).waitAt(0.4).end(0,20,180,0.0).build();

		assertEquals(originalSequence.getLastTime(), comparisonSequence.getLastTime(), 0.05);
	}
	/** Verifies that builder start time is ignored. */
	@Test
	public void builderStartTimeIsIgnored() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();
		double secondDuration = secondLeg().build().getLastTime();
		PathPoint lateStart = new PathPoint();
		lateStart.time = 10;
		lateStart.position = new Pos2D(40, 0, 0);
		lateStart.velocity = new Pos2D(0, 0, 0);

		sequence.addToSequence(builder().start(lateStart).splineTo(60, 0, 1).end(80, 0, 0));

		assertEquals(firstDuration + secondDuration, sequence.getLastTime(), 0.05);
	}
	/** Verifies that builder start position is replaced by the end of the sequence. */
	@Test
	public void builderStartPositionIsReplacedByTheEndOfTheSequence() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();

		sequence.addToSequence(builder().start(50, 10, 0).splineTo(60, 0, 1).end(80, 0, 0));

		Pos2D join = sequence.getMovement(firstDuration + 0.01).position;
		assertEquals(40, join.x, 0.5);
		assertEquals(0, join.y, 0.5);
		PathTestSupport.assertFinitePoseAndVelocity(sequence);
	}
	/** Verifies that builder start velocity is replaced by the end of the sequence. */
	@Test
	public void builderStartVelocityIsReplacedByTheEndOfTheSequence() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();
		PathPoint movingStart = new PathPoint();
		movingStart.time = 0;
		movingStart.position = new Pos2D(40, 0, 0);
		movingStart.velocity = new Pos2D(30, 0, 0);

		sequence.addToSequence(builder().start(movingStart).splineTo(60, 0, 1).end(80, 0, 0));

		PathTestSupport.assertFinitePoseAndVelocity(sequence);
		assertEquals(0, PathTestSupport.speed(sequence.getMovement(firstDuration)), 1.0);
	}
	/** Verifies that action indices are shifted past the existing markers. */
	@Test
	public void actionIndicesAreShiftedPastTheExistingMarkers() {
		Sequence sequence = firstLeg();
		double firstDuration = sequence.getLastTime();
		FlagAction action = new FlagAction(new Pos2D(60, 0, 0));
		sequence.addToSequence(secondLeg().addAction(action));

		sequence.updateActions(packetAt(60, 0), firstDuration - 0.1);

		assertEquals("Action ran during the first leg", 0, action.executeCount);
	}
}
