package com.broombots.sweep.Classes;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Unusual action lifecycles driven through Robot.runAction and Robot.updateActions. */
public class RobotActionEdgeCaseTest {
	private static class TestRobot extends Robot {
	}

	private static class LoggingAction extends Action {
		final String name;
		final ArrayList<String> log;
		boolean complete = true;

		LoggingAction(Robot robot, String name, ArrayList<String> log) {
			super(robot);
			this.name = name;
			this.log = log;
		}

		@Override
		public void execute() {
			log.add(name + ".execute");
		}

		@Override
		public void process() {
			log.add(name + ".process");
		}

		@Override
		public boolean completion() {
			return complete;
		}

		@Override
		public void end() {
			log.add(name + ".end");
		}
	}

	@Test
	public void actionLifecycleRunsInOrder() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();

		robot.runAction(new LoggingAction(robot, "A", log));
		robot.updateActions();

		assertEquals(Arrays.asList("A.execute", "A.process", "A.end"), log);
	}

	@Test
	public void updatingWithNoActionsDoesNothing() {
		new TestRobot().updateActions();
	}

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

	@Test
	public void negativeHoldTimeCompletesImmediately() {
		TestRobot robot = new TestRobot();
		Action action = new Action(robot) {
			@Override
			public void execute() {
			}

			@Override
			public void process() {
			}

			@Override
			public void end() {
			}
		};
		action.setHoldTime(-1);

		assertTrue(action.completion());
	}

	@Test
	public void actionCanStartAnotherActionFromItsEnd() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction second = new LoggingAction(robot, "second", log);
		LoggingAction first = new LoggingAction(robot, "first", log) {
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

	@Test
	public void actionCanStartAnotherActionFromProcess() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction spawned = new LoggingAction(robot, "spawned", log);
		LoggingAction spawner = new LoggingAction(robot, "spawner", log) {
			boolean spawnedOnce;

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

	private static int Collections(ArrayList<String> log, String entry) {
		return java.util.Collections.frequency(log, entry);
	}
}
