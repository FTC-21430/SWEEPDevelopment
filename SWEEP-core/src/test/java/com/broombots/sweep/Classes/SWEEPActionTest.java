package com.broombots.sweep.Classes;

import com.broombots.sweep.Builder.Sequence;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Tests SWEEP action pose, tolerance, path-time trigger, and hold-time behavior. */
public class SWEEPActionTest {
	/** Verifies that trigger position can be set and checked. */
	@Test
	public void triggerPositionCanBeSetAndChecked() {
		TestSWEEPAction action = new TestSWEEPAction(new TestRobot());

		assertFalse(action.isPositionSet());
		action.setPosition(new Pos2D(10.0, 20.0, 0.0));

		assertTrue(action.isPositionSet());
		assertTrue(action.checkTrigger(packetAt(10.0, 20.0), 0.0));
	}
	/** Verifies that constructor uses configured tolerance and includes boundary. */
	@Test
	public void constructorUsesConfiguredToleranceAndIncludesBoundary() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 2.0, 5.0, 0.0
		);

		assertTrue(action.isPositionSet());
		assertTrue(action.checkTrigger(packetAt(-3.0, -4.0), 0.0));
		assertFalse(action.checkTrigger(packetAt(3.0, -4.001), 0.0));
		assertTrue(action.checkTrigger(packetAt(3.0, 4.0), 0.0));
		assertFalse(action.checkTrigger(packetAt(3.0, 4.01), 0.0));
	}
	/** Verifies that setting null position is rejected. */
	@Test
	public void settingNullPositionIsRejected() {
		TestSWEEPAction action = new TestSWEEPAction(new TestRobot());

		try {
			action.setPosition(null);
			fail("Expected NullPointerException");
		} catch (NullPointerException expected) {
			assertFalse(action.isPositionSet());
		}
	}
	/** Verifies that hold time starts when action is triggered not when created. */
	@Test
	public void holdTimeStartsWhenActionIsTriggeredNotWhenCreated() throws InterruptedException {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 0.3, 1.0, 0.0
		);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());

		Thread.sleep(400); // longer than the hold time, before the trigger
		sequence.updateActions(packetAt(0.0, 0.0), 0.0);
		assertEquals(1, action.executeCount);

		sequence.updateActions(packetAt(0.0, 0.0), 0.0);
		assertEquals(0, action.endCount);
		assertEquals(1, action.processCount);
	}
	/** Verifies that triggered action ends once hold time elapses. */
	@Test
	public void triggeredActionEndsOnceHoldTimeElapses() throws InterruptedException {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 0.2, 1.0, 0.0
		);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());

		sequence.updateActions(packetAt(0.0, 0.0), 0.0);
		Thread.sleep(300);
		sequence.updateActions(packetAt(0.0, 0.0), 0.0);

		assertEquals(1, action.endCount);
	}
	/** Verifies that action outside tolerance is not triggered. */
	@Test
	public void actionOutsideToleranceIsNotTriggered() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 0.2, 1.0, 0.0
		);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());

		sequence.updateActions(packetAt(50.0, 50.0), 0.0);

		assertEquals(0, action.executeCount);
	}
	/** Verifies that action run by robot holds from when it was run. */
	@Test
	public void actionRunByRobotHoldsFromWhenItWasRun() throws InterruptedException {
		TestRobot robot = new TestRobot();
		TestSWEEPAction action = new TestSWEEPAction(robot, new Pos2D(0.0, 0.0, 0.0), 0.3, 1.0, 0.0);

		Thread.sleep(400);
		robot.runAction(action);

		assertFalse(action.completion());
	}
	/** Verifies that action waits until its wait time even inside tolerance. */
	@Test
	public void actionWaitsUntilItsWaitTimeEvenInsideTolerance() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 1.0, 1.0, 2.5
		);

		assertFalse(action.checkTrigger(packetAt(0.0, 0.0), 0.0));
		assertFalse(action.checkTrigger(packetAt(0.0, 0.0), 2.49));
		assertTrue(action.checkTrigger(packetAt(0.0, 0.0), 2.5));
		assertTrue(action.checkTrigger(packetAt(0.0, 0.0), 10.0));
	}
	/** Verifies that action past its wait time still needs to be in tolerance. */
	@Test
	public void actionPastItsWaitTimeStillNeedsToBeInTolerance() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 1.0, 1.0, 2.5
		);

		assertFalse(action.checkTrigger(packetAt(50.0, 50.0), 10.0));
	}
	/** Verifies that set wait time changes when the action can trigger. */
	@Test
	public void setWaitTimeChangesWhenTheActionCanTrigger() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 1.0, 1.0, 0.0
		);

		assertTrue(action.checkTrigger(packetAt(0.0, 0.0), 1.0));
		action.setWaitTime(5.0);
		assertFalse(action.checkTrigger(packetAt(0.0, 0.0), 1.0));
	}
	/** Verifies that sequence holds action until wait time passes then triggers it. */
	@Test
	public void sequenceHoldsActionUntilWaitTimePassesThenTriggersIt() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 1.0, 1.0, 3.0
		);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());

		sequence.updateActions(packetAt(0.0, 0.0), 1.0);
		assertEquals(0, action.executeCount);

		sequence.updateActions(packetAt(0.0, 0.0), 3.0);
		assertEquals(1, action.executeCount);
	}
	/** Verifies that wait time does not shorten or extend hold time. */
	@Test
	public void waitTimeDoesNotShortenOrExtendHoldTime() throws InterruptedException {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 0.2, 1.0, 3.0
		);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());

		sequence.updateActions(packetAt(0.0, 0.0), 3.0);
		Thread.sleep(300);
		sequence.updateActions(packetAt(0.0, 0.0), 3.3);

		assertEquals(1, action.endCount);
	}
	/**
	 * Creates a localization packet at the supplied coordinates.
	 * @param x x coordinate in inches
	 * @param y y coordinate in inches
	 * @return localization packet at the requested position
	 */
	private static LocalizationPacket packetAt(double x, double y) {
		return new LocalizationPacket(x, y, 0.0, 0.0, 0.0);
	}

	private static class TestRobot extends Robot {
	}

	private static class TestSWEEPAction extends SWEEPAction {
		int executeCount;
		int processCount;
		int endCount;

		/** Creates a test action associated with the supplied robot. */
		TestSWEEPAction(Robot robot) {
			super(robot);
		}

		/**
		 * Creates a test action with configured trigger and timing values.
		 * @param robot robot that owns the action
		 * @param triggerPosition trigger position in inches and heading in degrees
		 * @param holdTime action hold duration in seconds
		 * @param triggerTolerance trigger radius in inches
		 * @param waitTime earliest trigger time in seconds
		 */
		TestSWEEPAction(Robot robot, Pos2D triggerPosition, double holdTime, double triggerTolerance,
										double waitTime) {
			super(robot, triggerPosition, holdTime, triggerTolerance, waitTime);
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
			executeCount++;
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
			processCount++;
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			endCount++;
		}
	}
}
