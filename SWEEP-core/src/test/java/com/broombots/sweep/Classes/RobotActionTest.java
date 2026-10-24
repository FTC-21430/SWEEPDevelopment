package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class RobotActionTest {
	@Test
	public void runActionExecutesAndUpdateProcessesAndEndsCompletedAction() {
		Robot robot = new TestRobot();
		CountingAction action = new CountingAction(robot);

		robot.runAction(action);
		assertEquals(1, action.executeCount);

		robot.updateActions();
		assertEquals(1, action.processCount);
		assertEquals(1, action.endCount);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void updateProcessesAndRemovesEveryCompletedAction() {
		Robot robot = new TestRobot();
		CountingAction firstAction = new CountingAction(robot);
		CountingAction secondAction = new CountingAction(robot);
		robot.runAction(firstAction);
		robot.runAction(secondAction);

		robot.updateActions();

		assertEquals(1, firstAction.processCount);
		assertEquals(1, firstAction.endCount);
		assertEquals(1, secondAction.processCount);
		assertEquals(1, secondAction.endCount);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void runActionIgnoresNull() {
		Robot robot = new TestRobot();

		robot.runAction(null);

		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void actionRemainsActiveUntilItsHoldTimeHasElapsed() {
		CountingAction action = new CountingAction(new TestRobot());
		action.setHoldTime(60.0);

		assertFalse(action.completion());
	}

	@Test
	public void interruptEndsActionAndAllowsInterruption() {
		CountingAction action = new CountingAction(new TestRobot());

		assertTrue(action.interrupt());
		assertEquals(1, action.endCount);
	}

	@Test
	public void interruptBlocksForTwentyIterationsThenReleasesControl() {
		HoldingAction action = new HoldingAction(new TestRobot(), 20);

		for (int i = 0; i < 20; i++) {
			assertFalse("Interrupt should be blocked on iteration " + i, action.interrupt());
			assertEquals(0, action.endCount);
		}

		assertTrue(action.interrupt());
		assertEquals(1, action.endCount);
	}

	private static class TestRobot extends Robot {
	}

	private static class CountingAction extends Action {
		int executeCount;
		int processCount;
		int endCount;

		CountingAction(Robot robot) {
			super(robot);
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

	private static class HoldingAction extends CountingAction {
		private int blockedInterrupts;
		private final int iterationsToHold;

		HoldingAction(Robot robot, int iterationsToHold) {
			super(robot);
			this.iterationsToHold = iterationsToHold;
		}

		@Override
		public boolean interrupt() {
			if (blockedInterrupts < iterationsToHold) {
				blockedInterrupts++;
				return false;
			}
			return super.interrupt();
		}
	}
}
