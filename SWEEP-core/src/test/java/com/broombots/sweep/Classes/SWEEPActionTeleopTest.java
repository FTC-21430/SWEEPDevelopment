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
/** Tests SWEEP action behavior when managed through teleop robot action flow. */
public class SWEEPActionTeleopTest {
	/** Counts telemetry calls; every other Telemetry method is a no-op. */
	private static class TelemetryCounter {
		int updates;
		int lines;
	}

	private static class TestRobot extends Robot {
		final TelemetryCounter counter = new TelemetryCounter();

		/** Creates a robot with telemetry call counters for tests. */
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

		/** Creates an action that records lifecycle events without a hold or wait. */
		LoggingAction(Robot robot, String name, ArrayList<String> log) {
			super(robot);
			this.name = name;
			this.log = log;
		}

		/** Creates an action that records lifecycle events with the supplied hold time. */
		LoggingAction(Robot robot, String name, ArrayList<String> log, double holdTime) {
			super(robot, holdTime);
			this.name = name;
			this.log = log;
		}

		/** Creates an action that records lifecycle events with hold and wait times. */
		LoggingAction(Robot robot, String name, ArrayList<String> log, double holdTime, double waitTime) {
			super(robot, holdTime, waitTime);
			this.name = name;
			this.log = log;
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
			executeCount++;
			log.add(name + ".execute");
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
			if (interrupting) return;
			processCount++;
			log.add(name + ".process");
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			endCount++;
			log.add(name + ".end");
		}
	}
	/**
	 * Pauses the test thread for the requested duration.
	 * @param millis pause duration in milliseconds
	 */
	private static void sleep(long millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}
	/** Verifies that action with only a robot runs without a trigger position. */
	@Test
	public void actionWithOnlyARobotRunsWithoutATriggerPosition() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>());

		robot.runAction(action);

		assertFalse(action.isPositionSet());
		assertEquals(1, action.executeCount);
	}
	/** Verifies that every teleop constructor executes immediately. */
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
	/** Verifies that wait time and trigger position do not delay ateleop action. */
	@Test
	public void waitTimeAndTriggerPositionDoNotDelayATeleopAction() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.0, 1000.0);
		action.setPosition(new Pos2D(500, 500, 0));

		robot.runAction(action);

		assertEquals(1, action.executeCount);
	}
	/** Verifies that zero hold action runs process then ends on the next update. */
	@Test
	public void zeroHoldActionRunsProcessThenEndsOnTheNextUpdate() {
		TestRobot robot = new TestRobot();
		ArrayList<String> log = new ArrayList<>();

		robot.runAction(new LoggingAction(robot, "A", log));
		robot.teleopUpdate();

		assertEquals(Arrays.asList("A.execute", "A.process", "A.end"), log);
		assertTrue(robot.activeActions.isEmpty());
	}
	/** Verifies that long hold action is processed every loop without ending. */
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
	/** Verifies that hold action ends once its hold time has passed. */
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
	/** Verifies that hold time counts from when the driver started the action. */
	@Test
	public void holdTimeCountsFromWhenTheDriverStartedTheAction() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.2);

		sleep(300);
		robot.runAction(action);
		robot.teleopUpdate();

		assertEquals("Hold time must not be used up by time spent before runAction", 0, action.endCount);
	}
	/** Verifies that actions with different hold times finish independently. */
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
	/** Verifies that queued driver actions run in the order they were started. */
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
	/** Verifies that same action can be run again after it finished. */
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
	/** Verifies that rerunning aheld action restarts its hold timer. */
	@Test
	public void rerunningAHeldActionRestartsItsHoldTimer() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 0.2);
		robot.runAction(action);
		sleep(250);

		action.resetTimer();

		assertFalse(action.completion());
	}
	/** Verifies that button pressed on one loop starts the action exactly once. */
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
	/** Verifies that action started from another actions end runs on later updates. */
	@Test
	public void actionStartedFromAnotherActionsEndRunsOnLaterUpdates() {
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
		robot.teleopUpdate();
		robot.teleopUpdate();

		assertEquals(Arrays.asList("first.execute", "first.process", "first.end", "second.execute",
				"second.process", "second.end"), log);
	}
	/** Verifies that many actions finish in asingle update. */
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
	/** Verifies that teleop update sends telemetry and updates actions. */
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
	/** Verifies that autonomous update also updates actions started directly. */
	@Test
	public void autonomousUpdateAlsoUpdatesActionsStartedDirectly() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>());
		robot.runAction(action);

		robot.autonomousUpdate();

		assertEquals(1, action.endCount);
		assertEquals(1, robot.counter.updates);
	}
	/** Verifies that interrupting arunning action ends it once. */
	@Test
	public void interruptingARunningActionEndsItOnce() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		robot.runAction(action);
		robot.teleopUpdate();

		assertTrue(action.interrupt());

		assertEquals(1, action.endCount);
	}
	/** Verifies that interrupted action is not ended again by later updates. */
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
	/** Verifies that actions on different robots do not share state. */
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

		/** Creates an action that records interruption and recovery events. */
		AbortAwareAction(Robot robot, double holdTime) {
			super(robot, holdTime);
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
		}
		/**
		 * Reports whether the simulated action accepts interruption.
		 * @return localization packet at the requested position
		 */
		@Override
		public boolean interrupt() {
			return !blockInterrupt && super.interrupt();
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			log.add(isInterrupted() ? "abort" : "finish");
		}
	}
	/** Verifies that end sees no interrupt when the action finishes naturally. */
	@Test
	public void endSeesNoInterruptWhenTheActionFinishesNaturally() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 0.0);
		robot.runAction(action);

		robot.teleopUpdate();

		assertEquals(Arrays.asList("finish"), action.log);
	}
	/** Verifies that end sees the interrupt when the robot interrupts the action. */
	@Test
	public void endSeesTheInterruptWhenTheRobotInterruptsTheAction() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 60.0);
		robot.runAction(action);

		assertTrue(robot.interruptAction(action));

		assertEquals(Arrays.asList("abort"), action.log);
		assertTrue(robot.activeActions.isEmpty());
	}
	/** Verifies that interrupted action is no longer processed. */
	@Test
	public void interruptedActionIsNoLongerProcessed() {
		TestRobot robot = new TestRobot();
		LoggingAction action = new LoggingAction(robot, "A", new ArrayList<String>(), 60.0);
		robot.runAction(action);
		robot.interruptAction(action);

		robot.teleopUpdate();

		assertEquals(0, action.processCount);
	}
	/** Verifies that direct interrupt is also dropped without processing or asecond end. */
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
	/** Verifies that blocked interrupt leaves the action running and not interrupted. */
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
	/** Verifies that blocked interrupt can be accepted later. */
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
	/** Verifies that interrupting an action that is not running does nothing. */
	@Test
	public void interruptingAnActionThatIsNotRunningDoesNothing() {
		TestRobot robot = new TestRobot();
		AbortAwareAction action = new AbortAwareAction(robot, 60.0);

		assertFalse(robot.interruptAction(action));
		assertFalse(robot.interruptAction(null));

		assertTrue(action.log.isEmpty());
	}
	/** Verifies that rerunning an interrupted action starts clean and finishes normally. */
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
	/** Verifies that interrupting twice only ends once. */
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

		/** Creates a simulated arm action with the supplied hold time. */
		ArmAction(Robot robot, double holdTime) {
			super(robot, holdTime);
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
			if (isInterrupted()) {
				recoveryLoops++;
				log.add("recover");
			}
		}
		/**
		 * Reports whether the simulated action is safe to release.
		 * @return localization packet at the requested position
		 */
		@Override
		protected boolean isSafeToRelease() {
			return armSafe;
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			log.add(isInterrupted() ? "end-interrupted" : "end");
		}
	}
	/** Verifies that recovering action stays active and keeps being processed. */
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
	/** Verifies that recovering action is released only once the sensor says it is safe. */
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
	/** Verifies that finished hold time cannot cut recovery short. */
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
	/** Verifies that arm already safe is released immediately on interrupt. */
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
	/** Verifies that interrupting again while recovering does not restart or end it. */
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
	/** Verifies that non interrupted action ignores the safe to release check. */
	@Test
	public void nonInterruptedActionIgnoresTheSafeToReleaseCheck() {
		TestRobot robot = new TestRobot();
		ArmAction arm = new ArmAction(robot, 0.0);
		robot.runAction(arm);

		robot.teleopUpdate();

		assertEquals(Arrays.asList("end"), arm.log);
	}
	/** Verifies that other actions keep running while one recovers. */
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
	/** Verifies that recovered action can be run again as afresh run. */
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
	/** Verifies that recovery has no timeout by default. */
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
	/** Verifies that recovery timeout force releases an arm that never reports safe. */
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
	/** Verifies that end can tell that recovery timed out. */
	@Test
	public void endCanTellThatRecoveryTimedOut() {
		TestRobot robot = new TestRobot();
		final boolean[] timedOut = new boolean[1];
		ArmAction arm = new ArmAction(robot, 60.0) {
			/** Ends the simulated action. */
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
	/** Verifies that sensor release before the timeout is not atimeout. */
	@Test
	public void sensorReleaseBeforeTheTimeoutIsNotATimeout() {
		TestRobot robot = new TestRobot();
		final boolean[] timedOut = new boolean[1];
		ArmAction arm = new ArmAction(robot, 60.0) {
			/** Ends the simulated action. */
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
	/** Verifies that timeout does not apply to an action that is not interrupted. */
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
	/** Verifies that interrupting again does not restart the timeout. */
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
	/** Verifies that timeout of zero or less means no limit. */
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
	/** Verifies that rerun clears the previous recovery timeout. */
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
