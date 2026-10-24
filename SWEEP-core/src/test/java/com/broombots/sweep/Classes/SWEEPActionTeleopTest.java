package com.broombots.sweep.Classes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * SWEEPActions used the way teleop uses them: started directly with Robot.runAction from driver input,
 * with no path, no trigger position and no localization packets.
 */
public class SWEEPActionTeleopTest {
	/** Counts telemetry calls; every other Telemetry method is a no-op. */
	private static class TelemetryCounter {
		int updates;
		int lines;
	}

	private static class TestRobot extends Robot {
		final TelemetryCounter counter = new TelemetryCounter();

		TestRobot() {
			telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
					new Class<?>[]{Telemetry.class}, (proxy, method, args) -> {
						if (method.getName().equals("update")) counter.updates++;
						if (method.getName().equals("addLine")) counter.lines++;
						Class<?> type = method.getReturnType();
						return type == boolean.class ? (Object) false : null;
					});
		}
	}

	private static class LoggingAction extends SWEEPAction {
		final String name;
		final ArrayList<String> log;
		int executeCount;
		int processCount;
		int endCount;

		LoggingAction(Robot robot, String name, ArrayList<String> log) {
			super(robot);
			this.name = name;
			this.log = log;
		}

		LoggingAction(Robot robot, String name, ArrayList<String> log, double holdTime) {
			super(robot, holdTime);
			this.name = name;
			this.log = log;
		}

		LoggingAction(Robot robot, String name, ArrayList<String> log, double holdTime, double waitTime) {
			super(robot, holdTime, waitTime);
			this.name = name;
			this.log = log;
		}

		@Override
		public void execute() {
			executeCount++;
			log.add(name + ".execute");
		}

		@Override
		public void process() {
			if (interrupting) return;
			processCount++;
			log.add(name + ".process");
		}

		@Override
		public void end() {
			endCount++;
			log.add(name + ".end");
		}
	}

	private static void sleep(long millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}

	@Test
	public void actionWithOnlyARobotRunsWithoutATriggerPosition() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>());

		robot.runAction(action);

		assertFalse(action.isPositionSet());
		assertEquals(1, action.executeCount);
	}

	@Test
	public void everyTeleopConstructorExecutesImmediately() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction plain = new LoggingAction(robot, "plain", log);
		LoggingAction held = new LoggingAction(robot, "held", log, 5.0);
		LoggingAction waited = new LoggingAction(robot, "waited", log, 5.0, 5.0);
		LoggingAction positioned = new LoggingAction(robot, "positioned", log);
		positioned.setPosition(new Pos2D(100, 100, 0));

		robot.runAction(plain);
		robot.runAction(held);
		robot.runAction(waited);
		robot.runAction(positioned);

		assertEquals(Arrays.asList("plain.execute", "held.execute", "waited.execute", "positioned.execute"), log);
	}

	@Test
	public void waitTimeAndTriggerPositionDoNotDelayATeleopAction() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.0, 1000.0);
		action.setPosition(new Pos2D(500, 500, 0));

		robot.runAction(action);

		assertEquals(1, action.executeCount);
	}

	@Test
	public void zeroHoldActionRunsProcessThenEndsOnTheNextUpdate() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();

		robot.runAction(new LoggingAction(robot, "A", log));
		robot.teleopUpdate();

		assertEquals(Arrays.asList("A.execute", "A.process", "A.end"), log);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void longHoldActionIsProcessedEveryLoopWithoutEnding() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);

		robot.runAction(action);
		for (int i = 0; i < 25; i++) {
			robot.teleopUpdate();
		}

		assertEquals(25, action.processCount);
		assertEquals(0, action.endCount);
		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void holdActionEndsOnceItsHoldTimeHasPassed() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.1);

		robot.runAction(action);
		robot.teleopUpdate();
		assertEquals(0, action.endCount);

		sleep(150);
		robot.teleopUpdate();

		assertEquals(1, action.endCount);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void holdTimeCountsFromWhenTheDriverStartedTheAction() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.2);

		sleep(300);
		robot.runAction(action);
		robot.teleopUpdate();

		assertEquals("Hold time must not be used up by time spent before runAction", 0, action.endCount);
	}

	@Test
	public void actionsWithDifferentHoldTimesFinishIndependently() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();
		LoggingAction quick = new LoggingAction(robot, "quick", log, 0.0);
		LoggingAction slow = new LoggingAction(robot, "slow", log, 60.0);

		robot.runAction(slow);
		robot.runAction(quick);
		robot.teleopUpdate();

		assertEquals(1, quick.endCount);
		assertEquals(0, slow.endCount);
		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void queuedDriverActionsRunInTheOrderTheyWereStarted() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();

		robot.runAction(new LoggingAction(robot, "A", log));
		robot.runAction(new LoggingAction(robot, "B", log));
		robot.runAction(new LoggingAction(robot, "C", log));
		robot.teleopUpdate();

		assertEquals(Arrays.asList("A.execute", "B.execute", "C.execute",
				"A.process", "A.end", "B.process", "B.end", "C.process", "C.end"), log);
	}

	@Test
	public void sameActionCanBeRunAgainAfterItFinished() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>());

		robot.runAction(action);
		robot.teleopUpdate();
		robot.runAction(action);
		robot.teleopUpdate();

		assertEquals(2, action.executeCount);
		assertEquals(2, action.endCount);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void rerunningAHeldActionRestartsItsHoldTimer() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.2);
		robot.runAction(action);
		sleep(250);

		action.resetTimer();

		assertFalse(action.completion());
	}

	@Test
	public void buttonPressedOnOneLoopStartsTheActionExactlyOnce() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		boolean wasPressed = false;

		for (int loop = 0; loop < 10; loop++) {
			boolean pressed = loop >= 3 && loop <= 6;
			if (pressed && !wasPressed) robot.runAction(action);
			wasPressed = pressed;
			robot.teleopUpdate();
		}

		assertEquals(1, action.executeCount);
		assertEquals(7, action.processCount);
	}

	@Test
	public void actionStartedFromAnotherActionsEndRunsOnLaterUpdates() {
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
		robot.teleopUpdate();
		robot.teleopUpdate();

		assertEquals(Arrays.asList("first.execute", "first.process", "first.end", "second.execute",
				"second.process", "second.end"), log);
	}

	@Test
	public void manyActionsFinishInASingleUpdate() {
		TestRobot robot = new TestRobot();
		ArrayList<LoggingAction> actions = new ArrayList<>();
		for (int i = 0; i < 200; i++) {
			LoggingAction action = new LoggingAction(robot, "A" + i, new ArrayList<String>());
			actions.add(action);
			robot.runAction(action);
		}

		robot.teleopUpdate();

		for (LoggingAction action : actions) {
			assertEquals(1, action.endCount);
		}
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void teleopUpdateSendsTelemetryAndUpdatesActions() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>());
		robot.runAction(action);

		robot.teleopUpdate();

		assertEquals(1, robot.counter.updates);
		assertTrue(robot.counter.lines > 0);
		assertEquals(1, action.endCount);
	}

	@Test
	public void autonomousUpdateAlsoUpdatesActionsStartedDirectly() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>());
		robot.runAction(action);

		robot.autonomousUpdate();

		assertEquals(1, action.endCount);
		assertEquals(1, robot.counter.updates);
	}

	@Test
	public void interruptingARunningActionEndsItOnce() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		robot.runAction(action);
		robot.teleopUpdate();

		assertTrue(action.interrupt());

		assertEquals(1, action.endCount);
	}

	@Test
	public void interruptedActionIsNotEndedAgainByLaterUpdates() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.1);
		robot.runAction(action);
		action.interrupt();

		sleep(150);
		robot.teleopUpdate();

		assertEquals("end() ran for the interrupt and again when the hold time elapsed", 1, action.endCount);
	}

	@Test
	public void actionsOnDifferentRobotsDoNotShareState() {
		TestRobot first = new TestRobot();
		TestRobot second = new TestRobot();
		LoggingAction action = new LoggingAction(first, "A", new ArrayList<String>(), 60.0);

		first.runAction(action);

		assertEquals(1, first.activeActions.size());
		assertTrue(second.activeActions.isEmpty());
	}

	private static class AbortAwareAction extends SWEEPAction {
		final ArrayList<String> log = new ArrayList<>();
		boolean blockInterrupt;

		AbortAwareAction(Robot robot, double holdTime) {
			super(robot, holdTime);
		}

		@Override
		public void execute() {
		}

		@Override
		public void process() {
		}

		@Override
		public boolean interrupt() {
			return !blockInterrupt && super.interrupt();
		}

		@Override
		public void end() {
			log.add(isInterrupted() ? "abort" : "finish");
		}
	}

	@Test
	public void endSeesNoInterruptWhenTheActionFinishesNaturally() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 0.0);
		robot.runAction(action);

		robot.teleopUpdate();

		assertEquals(Arrays.asList("finish"), action.log);
	}

	@Test
	public void endSeesTheInterruptWhenTheRobotInterruptsTheAction() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 60.0);
		robot.runAction(action);

		assertTrue(robot.interruptAction(action));

		assertEquals(Arrays.asList("abort"), action.log);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void interruptedActionIsNoLongerProcessed() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		robot.runAction(action);
		robot.interruptAction(action);

		robot.teleopUpdate();

		assertEquals(0, action.processCount);
	}

	@Test
	public void directInterruptIsAlsoDroppedWithoutProcessingOrASecondEnd() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		robot.runAction(action);
		action.interrupt();

		robot.teleopUpdate();

		assertEquals(0, action.processCount);
		assertEquals(1, action.endCount);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void blockedInterruptLeavesTheActionRunningAndNotInterrupted() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 60.0);
		action.blockInterrupt = true;
		robot.runAction(action);

		assertFalse(robot.interruptAction(action));

		assertTrue(action.log.isEmpty());
		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void blockedInterruptCanBeAcceptedLater() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 60.0);
		action.blockInterrupt = true;
		robot.runAction(action);
		robot.interruptAction(action);

		action.blockInterrupt = false;

		assertTrue(robot.interruptAction(action));
		assertEquals(Arrays.asList("abort"), action.log);
	}

	@Test
	public void interruptingAnActionThatIsNotRunningDoesNothing() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 60.0);

		assertFalse(robot.interruptAction(action));
		assertFalse(robot.interruptAction(null));

		assertTrue(action.log.isEmpty());
	}

	@Test
	public void rerunningAnInterruptedActionStartsCleanAndFinishesNormally() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 0.0);
		robot.runAction(action);
		robot.interruptAction(action);

		robot.runAction(action);
		robot.teleopUpdate();

		assertEquals(Arrays.asList("abort", "finish"), action.log);
	}

	@Test
	public void interruptingTwiceOnlyEndsOnce() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		robot.runAction(action);

		robot.interruptAction(action);
		action.interrupt();

		assertEquals(1, action.endCount);
	}

	/** Arm that must be driven back to a safe position after an interrupt, reported by a (fake) sensor. */
	private static class ArmAction extends SWEEPAction {
		final ArrayList<String> log = new ArrayList<>();
		boolean armSafe;
		int recoveryLoops;

		ArmAction(Robot robot, double holdTime) {
			super(robot, holdTime);
		}

		@Override
		public void execute() {
		}

		@Override
		public void process() {
			if (isInterrupted()) {
				recoveryLoops++;
				log.add("recover");
			}
		}

		@Override
		protected boolean isSafeToRelease() {
			return armSafe;
		}

		@Override
		public void end() {
			log.add(isInterrupted() ? "end-interrupted" : "end");
		}
	}

	@Test
	public void recoveringActionStaysActiveAndKeepsBeingProcessed() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		robot.runAction(arm);

		assertTrue(robot.interruptAction(arm));
		robot.teleopUpdate();
		robot.teleopUpdate();
		robot.teleopUpdate();

		assertEquals(3, arm.recoveryLoops);
		assertEquals(1, robot.activeActions.size());
		assertFalse(arm.log.contains("end-interrupted"));
	}

	@Test
	public void recoveringActionIsReleasedOnlyOnceTheSensorSaysItIsSafe() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		robot.runAction(arm);
		robot.interruptAction(arm);
		robot.teleopUpdate();
		robot.teleopUpdate();

		arm.armSafe = true;
		robot.teleopUpdate();

		assertEquals(Arrays.asList("recover", "recover", "recover", "end-interrupted"), arm.log);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void finishedHoldTimeCannotCutRecoveryShort() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 0.05);
		robot.runAction(arm);
		robot.interruptAction(arm);

		sleep(100);
		robot.teleopUpdate();

		assertEquals(1, robot.activeActions.size());
		assertFalse(arm.log.contains("end-interrupted"));
	}

	@Test
	public void armAlreadySafeIsReleasedImmediatelyOnInterrupt() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		arm.armSafe = true;
		robot.runAction(arm);

		assertTrue(robot.interruptAction(arm));

		assertEquals(Arrays.asList("end-interrupted"), arm.log);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void interruptingAgainWhileRecoveringDoesNotRestartOrEndIt() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		robot.runAction(arm);
		robot.interruptAction(arm);

		assertTrue(robot.interruptAction(arm));
		assertTrue(robot.interruptAction(arm));

		assertTrue(arm.log.isEmpty());
		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void nonInterruptedActionIgnoresTheSafeToReleaseCheck() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 0.0);
		robot.runAction(arm);

		robot.teleopUpdate();

		assertEquals(Arrays.asList("end"), arm.log);
	}

	@Test
	public void otherActionsKeepRunningWhileOneRecovers() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		LoggingAction other = new LoggingAction(robot, "other", new ArrayList<String>(), 60.0);
		robot.runAction(arm);
		robot.runAction(other);
		robot.interruptAction(arm);

		robot.teleopUpdate();
		robot.teleopUpdate();

		assertEquals(2, other.processCount);
		assertEquals(2, robot.activeActions.size());
	}

	@Test
	public void recoveredActionCanBeRunAgainAsAFreshRun() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 0.0);
		robot.runAction(arm);
		robot.interruptAction(arm);
		arm.armSafe = true;
		robot.teleopUpdate();

		robot.runAction(arm);
		robot.teleopUpdate();

		assertEquals("end-interrupted", arm.log.get(arm.log.size() - 2 < 0 ? 0 : arm.log.size() - 2));
		assertEquals("end", arm.log.get(arm.log.size() - 1));
	}

	@Test
	public void recoveryHasNoTimeoutByDefault() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		robot.runAction(arm);
		robot.interruptAction(arm);

		sleep(100);
		robot.teleopUpdate();

		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void recoveryTimeoutForceReleasesAnArmThatNeverReportsSafe() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		arm.setRecoveryTimeout(0.1);
		robot.runAction(arm);
		robot.interruptAction(arm);
		robot.teleopUpdate();
		assertEquals(1, robot.activeActions.size());

		sleep(150);
		robot.teleopUpdate();

		assertTrue(robot.activeActions.isEmpty());
		assertEquals("end-interrupted", arm.log.get(arm.log.size() - 1));
	}

	@Test
	public void endCanTellThatRecoveryTimedOut() {
		TestRobot robot = new TestRobot();
		final boolean[] timedOut = new boolean[1];
		ArmAction arm = new ArmAction(robot, 60.0) {
			@Override
			public void end() {
				timedOut[0] = didRecoveryTimeOut();
				super.end();
			}
		};
		arm.setRecoveryTimeout(0.05);
		robot.runAction(arm);
		robot.interruptAction(arm);

		sleep(100);
		robot.teleopUpdate();

		assertTrue(timedOut[0]);
	}

	@Test
	public void sensorReleaseBeforeTheTimeoutIsNotATimeout() {
		TestRobot robot = new TestRobot();
		final boolean[] timedOut = new boolean[1];
		ArmAction arm = new ArmAction(robot, 60.0) {
			@Override
			public void end() {
				timedOut[0] = didRecoveryTimeOut();
				super.end();
			}
		};
		arm.setRecoveryTimeout(60.0);
		robot.runAction(arm);
		robot.interruptAction(arm);
		arm.armSafe = true;

		robot.teleopUpdate();

		assertFalse(timedOut[0]);
		assertTrue(robot.activeActions.isEmpty());
	}

	@Test
	public void timeoutDoesNotApplyToAnActionThatIsNotInterrupted() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		arm.setRecoveryTimeout(0.01);
		robot.runAction(arm);

		sleep(50);
		robot.teleopUpdate();

		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void interruptingAgainDoesNotRestartTheTimeout() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		arm.setRecoveryTimeout(0.15);
		robot.runAction(arm);
		robot.interruptAction(arm);

		sleep(100);
		robot.interruptAction(arm);
		sleep(80);
		robot.teleopUpdate();

		assertTrue("Second interrupt restarted the timeout", robot.activeActions.isEmpty());
	}

	@Test
	public void timeoutOfZeroOrLessMeansNoLimit() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		arm.setRecoveryTimeout(-1);
		robot.runAction(arm);
		robot.interruptAction(arm);

		sleep(50);
		robot.teleopUpdate();

		assertEquals(1, robot.activeActions.size());
	}

	@Test
	public void rerunClearsThePreviousRecoveryTimeout() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 60.0);
		arm.setRecoveryTimeout(0.05);
		robot.runAction(arm);
		robot.interruptAction(arm);
		sleep(80);
		robot.teleopUpdate();

		robot.runAction(arm);
		robot.teleopUpdate();

		assertEquals(1, robot.activeActions.size());
	}
}
