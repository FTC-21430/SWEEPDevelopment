package com.broombots.sweep.Classes;

import com.broombots.sweep.Builder.Sequence;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class SWEEPActionTest {
	@Test
	public void triggerPositionCanBeSetAndChecked() {
		TestSWEEPAction action = new TestSWEEPAction(new TestRobot());

		assertFalse(action.isPositionSet());
		action.setPosition(new Pos2D(10.0, 20.0, 0.0));

		assertTrue(action.isPositionSet());
		assertTrue(action.checkTrigger(packetAt(10.0, 20.0), 0.0));
	}

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

	@Test
	public void actionOutsideToleranceIsNotTriggered() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 0.2, 1.0, 0.0
		);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());

		sequence.updateActions(packetAt(50.0, 50.0), 0.0);

		assertEquals(0, action.executeCount);
	}

	@Test
	public void actionRunByRobotHoldsFromWhenItWasRun() throws InterruptedException {
		TestRobot robot = new TestRobot();
		TestSWEEPAction action = new TestSWEEPAction(robot, new Pos2D(0.0, 0.0, 0.0), 0.3, 1.0, 0.0);

		Thread.sleep(400);
		robot.runAction(action);

		assertFalse(action.completion());
	}

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

	@Test
	public void actionPastItsWaitTimeStillNeedsToBeInTolerance() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 1.0, 1.0, 2.5
		);

		assertFalse(action.checkTrigger(packetAt(50.0, 50.0), 10.0));
	}

	@Test
	public void setWaitTimeChangesWhenTheActionCanTrigger() {
		TestSWEEPAction action = new TestSWEEPAction(
				new TestRobot(), new Pos2D(0.0, 0.0, 0.0), 1.0, 1.0, 0.0
		);

		assertTrue(action.checkTrigger(packetAt(0.0, 0.0), 1.0));
		action.setWaitTime(5.0);
		assertFalse(action.checkTrigger(packetAt(0.0, 0.0), 1.0));
	}

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

	private static LocalizationPacket packetAt(double x, double y) {
		return new LocalizationPacket(x, y, 0.0, 0.0, 0.0);
	}

	private static class TestRobot extends Robot {
	}

	private static class TestSWEEPAction extends SWEEPAction {
		int executeCount;
		int processCount;
		int endCount;

		TestSWEEPAction(Robot robot) {
			super(robot);
		}

		TestSWEEPAction(Robot robot, Pos2D triggerPosition, double holdTime, double triggerTolerance,
										double waitTime) {
			super(robot, triggerPosition, holdTime, triggerTolerance, waitTime);
		}

		@Override
		public void execute() {
			executeCount++;
		}

		@Override
		public void process() {
			processCount++;
		}

		@Override
		public void end() {
			endCount++;
		}
	}
}
