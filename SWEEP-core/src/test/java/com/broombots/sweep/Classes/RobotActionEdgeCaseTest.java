package com.broombots.sweep.Classes;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Tests unusual action lifecycle ordering, including actions started during processing or cleanup. */
public class RobotActionEdgeCaseTest {
	private static class TestRobot extends Robot {
	}

	private static class LoggingAction extends Action {
		final String name;
		final ArrayList<String> log;
		boolean complete = true;

		/** Creates a logging action that records lifecycle events in order. */
		LoggingAction(Robot robot, String name, ArrayList<String> log) {
			super(robot);
			this.name = name;
			this.log = log;
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
			log.add(name + ".execute");
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
			log.add(name + ".process");
		}
		/**
		 * Reports whether the simulated action has completed.
		 * @return localization packet at the requested position
		 */
		@Override
		public boolean completion() {
			return complete;
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			log.add(name + ".end");
		}
	}
	/** Verifies that action lifecycle runs in order. */
	@Test
	public void actionLifecycleRunsInOrder() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();

		robot.runAction(new LoggingAction(robot, "A", log));
		robot.updateActions();

		assertEquals(Arrays.asList("A.execute", "A.process", "A.end"), log);
	}
	/** Verifies that updating with no actions does nothing. */
	@Test
	public void updatingWithNoActionsDoesNothing() {
		new TestRobot().updateActions();
	}
	/** Verifies that actions finish independently of each other. */
	@Test
	public void actionsFinishIndependentlyOfEachOther() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction quick = new LoggingAction(robot, "quick", log);
		LoggingAction slow = new LoggingAction(robot, "slow", log);
		slow.complete = false;
		robot.runAction(quick);
		robot.runAction(slow);

		robot.updateActions();
		robot.updateActions();

		assertEquals(1, Collections(log, "quick.process"));
		assertEquals(2, Collections(log, "slow.process"));
		assertEquals(1, Collections(log, "quick.end"));
		assertEquals(0, Collections(log, "slow.end"));
		assertEquals(1, robot.activeActions.size());
	}
	/** Verifies that negative hold time completes immediately. */
	@Test
	public void negativeHoldTimeCompletesImmediately() {
		TestRobot robot = new TestRobot();
		Action action = new Action(robot) {
			/** Executes the simulated action. */
			@Override
			public void execute() {
			}
			/** Runs one simulated action update. */
			@Override
			public void process() {
			}
			/** Ends the simulated action. */
			@Override
			public void end() {
			}
		};
		action.setHoldTime(-1);

		assertTrue(action.completion());
	}
	/** Verifies that action can start another action from its end. */
	@Test
	public void actionCanStartAnotherActionFromItsEnd() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction second = new LoggingAction(robot, "second", log);
		LoggingAction first = new LoggingAction(robot, "first", log) {
			/** Ends the simulated action. */
			@Override
			public void end() {
				super.end();
				robot.runAction(second);
			}
		};
		robot.runAction(first);

		robot.updateActions();
		robot.updateActions();

		assertTrue("Chained action never ran: " + log, log.contains("second.execute"));
	}
	/** Verifies that action can start another action from process. */
	@Test
	public void actionCanStartAnotherActionFromProcess() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction spawned = new LoggingAction(robot, "spawned", log);
		LoggingAction spawner = new LoggingAction(robot, "spawner", log) {
			boolean spawnedOnce;
			/** Runs one simulated action update. */
			@Override
			public void process() {
				super.process();
				if (spawnedOnce) return;
				spawnedOnce = true;
				robot.runAction(spawned);
			}
		};
		robot.runAction(spawner);

		robot.updateActions();

		assertTrue("Spawned action never ran: " + log, log.contains("spawned.execute"));
	}
	/**
	 * Counts occurrences of an entry in the supplied log.
	 * @param log ordered action-event log
	 * @param entry action event to count
	 * @return localization packet at the requested position
	 */
	private static int Collections(ArrayList<String> log, String entry) {
		return java.util.Collections.frequency(log, entry);
	}
}
