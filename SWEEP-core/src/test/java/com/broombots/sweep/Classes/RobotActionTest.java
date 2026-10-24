package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Tests robot action start, processing, completion, and interruption lifecycles. */
public class RobotActionTest {
	/** Verifies that run action executes and update processes and ends completed action. */
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
	/** Verifies that update processes and removes every completed action. */
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
	/** Verifies that run action ignores null. */
	@Test
	public void runActionIgnoresNull() {
		Robot robot = new TestRobot();

		robot.runAction(null);

		assertTrue(robot.activeActions.isEmpty());
	}
	/** Verifies that action remains active until its hold time has elapsed. */
	@Test
	public void actionRemainsActiveUntilItsHoldTimeHasElapsed() {
		CountingAction action = new CountingAction(new TestRobot());
		action.setHoldTime(60.0);

		assertFalse(action.completion());
	}
	/** Verifies that interrupt ends action and allows interruption. */
	@Test
	public void interruptEndsActionAndAllowsInterruption() {
		CountingAction action = new CountingAction(new TestRobot());

		assertTrue(action.interrupt());
		assertEquals(1, action.endCount);
	}
	/** Verifies that interrupt blocks for twenty iterations then releases control. */
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

		/** Creates a counting action attached to the supplied test robot. */
		CountingAction(Robot robot) {
			super(robot);
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

	private static class HoldingAction extends CountingAction {
		private int blockedInterrupts;
		private final int iterationsToHold;

		/** Creates an action that rejects interruption for a fixed number of checks. */
		HoldingAction(Robot robot, int iterationsToHold) {
			super(robot);
			this.iterationsToHold = iterationsToHold;
		}
		/**
		 * Reports whether the simulated action accepts interruption.
		 * @return localization packet at the requested position
		 */
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
