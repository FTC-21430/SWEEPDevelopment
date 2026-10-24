package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Robot;
import com.broombots.sweep.Classes.SWEEPAction;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Exercises boundary cases in action queues, including invalid trigger inputs and completion order.
 * These checks protect the action scheduling that coordinates robot work with path progress.
 */
public class SequenceActionEdgeCaseTest {
	/** Shared action trigger position, in inches and degrees. */
	private static final Pos2D HERE = new Pos2D(10, 10, 0);

	/** Minimal robot instance for action tests that do not issue drivetrain commands. */
	private static class TestRobot extends Robot {
	}

	/** Action that records lifecycle calls so queue ordering and repetition can be asserted. */
	private static class CountingAction extends SWEEPAction {
		final String name;
		final ArrayList<String> log;
		int executeCount;
		int processCount;
		int endCount;

		/**
		 * Creates a lifecycle-recording action at a configured trigger point.
		 * @param name label added to lifecycle entries
		 * @param log destination for lifecycle entries
		 * @param position trigger position in inches and heading in degrees
		 * @param holdTime action hold duration in seconds
		 * @param tolerance trigger radius in inches
		 * @param waitTime earliest trigger time in seconds
		 * @param splineIndex waypoint marker index
		 */
		CountingAction(String name, ArrayList<String> log, Pos2D position, double holdTime, double tolerance,
					   double waitTime, int splineIndex) {
			super(new TestRobot(), position, holdTime, tolerance, waitTime);
			this.name = name;
			this.log = log;
			setSplineIndex(splineIndex);
		}

		/**
		 * Creates an action at the shared test position with the requested hold duration.
		 * @param name action label
		 * @param log destination for lifecycle entries
		 * @param holdTime action hold duration in seconds
		 * @return configured counting action
		 */
		static CountingAction at(String name, ArrayList<String> log, double holdTime) {
			return new CountingAction(name, log, HERE, holdTime, 1.0, 0.0, -1);
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
			processCount++;
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			endCount++;
			log.add(name + ".end");
		}
	}

	/**
	 * Creates a sequence with explicit waypoint-time markers and actions.
	 * @param markers waypoint times in seconds
	 * @param actions actions in queue order
	 * @return sequence used to exercise action scheduling
	 */
	private static Sequence sequenceOf(ArrayList<Double> markers, SWEEPAction... actions) {
		return new Sequence(new ArrayList<PathPoint>(), actions, markers);
	}

	/**
	 * Creates a mutable list of waypoint-time markers.
	 * @param times waypoint times in seconds
	 * @return marker list
	 */
	private static ArrayList<Double> markers(Double... times) {
		return new ArrayList<>(Arrays.asList(times));
	}

	/**
	 * Creates localization data at the supplied robot pose.
	 * @param position robot position in inches and heading in degrees
	 * @return localization packet with zero velocity
	 */
	private static LocalizationPacket packetAt(Pos2D position) {
		return new LocalizationPacket(position.x, position.y, position.angle, 0, 0);
	}

	/**
	 * Repeats an action update at one simulated pose and time.
	 * @param sequence sequence whose actions are updated
	 * @param position robot position in inches and heading in degrees
	 * @param time simulated path time in seconds
	 * @param times number of repeated updates
	 */
	private static void update(Sequence sequence, Pos2D position, double time, int times) {
		for (int i = 0; i < times; i++) {
			sequence.updateActions(packetAt(position), time);
		}
	}
	/** Verifies that updating with no actions is harmless. */
	@Test
	public void updatingWithNoActionsIsHarmless() {
		Sequence sequence = sequenceOf(markers(0.0));

		update(sequence, HERE, 0, 100);
	}
	/** Verifies that completed action is never triggered again. */
	@Test
	public void completedActionIsNeverTriggeredAgain() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = CountingAction.at("A", log, 0);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 0, 20);

		assertEquals(1, action.executeCount);
		assertEquals(1, action.endCount);
	}
	/** Verifies that queued actions at the same position run in order each once. */
	@Test
	public void queuedActionsAtTheSamePositionRunInOrderEachOnce() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction a = CountingAction.at("A", log, 0);
		CountingAction b = CountingAction.at("B", log, 0);
		CountingAction c = CountingAction.at("C", log, 0);
		Sequence sequence = sequenceOf(markers(0.0), a, b, c);

		update(sequence, HERE, 0, 30);

		assertEquals(Arrays.asList("A.execute", "A.end", "B.execute", "B.end", "C.execute", "C.end"), log);
	}
	/** Verifies that next action waits while the current action is still holding. */
	@Test
	public void nextActionWaitsWhileTheCurrentActionIsStillHolding() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction slow = CountingAction.at("slow", log, 60);
		CountingAction next = CountingAction.at("next", log, 0);
		Sequence sequence = sequenceOf(markers(0.0), slow, next);

		update(sequence, HERE, 0, 50);

		assertEquals(0, next.executeCount);
		assertEquals(49, slow.processCount);
		assertEquals(0, slow.endCount);
	}
	/** Verifies that action ends before the next one starts. */
	@Test
	public void actionEndsBeforeTheNextOneStarts() {
		ArrayList<String> log = new ArrayList<>();
		Sequence sequence = sequenceOf(markers(0.0), CountingAction.at("A", log, 0), CountingAction.at("B", log, 0));

		update(sequence, HERE, 0, 10);

		assertTrue(log.indexOf("A.end") < log.indexOf("B.execute"));
	}
	/** Verifies that trigger position with na nnever triggers. */
	@Test
	public void triggerPositionWithNaNNeverTriggers() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, new Pos2D(Double.NaN, 0, 0), 0, 1.0, 0, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 0, 20);

		assertEquals(0, action.executeCount);
	}

	/** Confirms a positive-infinite wait remains unsatisfied at a large finite path time. */
	@Test
	public void infiniteWaitTimeNeverTriggers() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, Double.POSITIVE_INFINITY, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 1e9, 5);

		assertEquals(0, action.executeCount);
	}
	/** Verifies that negative wait time triggers immediately. */
	@Test
	public void negativeWaitTimeTriggersImmediately() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, -5.0, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 0, 1);

		assertEquals(1, action.executeCount);
	}
	/** Verifies that zero tolerance needs the exact position. */
	@Test
	public void zeroToleranceNeedsTheExactPosition() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 0.0, 0.0, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, new Pos2D(10.0001, 10, 0), 0, 1);
		assertEquals(0, action.executeCount);

		update(sequence, HERE, 0, 1);
		assertEquals(1, action.executeCount);
	}
	/** Verifies that waypoint index past the last marker uses the last marker. */
	@Test
	public void waypointIndexPastTheLastMarkerUsesTheLastMarker() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, 0.0, 99);
		Sequence sequence = sequenceOf(markers(0.0, 2.0, 4.0), action);

		update(sequence, HERE, 3.9, 1);
		assertEquals(0, action.executeCount);

		update(sequence, HERE, 4.0, 1);
		assertEquals(1, action.executeCount);
	}
	/** Verifies that actions without any markers still trigger. */
	@Test
	public void actionsWithoutAnyMarkersStillTrigger() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, 0.0, 3);
		Sequence sequence = sequenceOf(markers(), action);

		update(sequence, HERE, 0, 1);

		assertEquals(1, action.executeCount);
	}
	/** Verifies that null markers are rejected like the other arguments. */
	@Test
	public void nullMarkersAreRejectedLikeTheOtherArguments() {
		try {
			new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[0], null);
			fail("Expected IllegalArgumentException");
		} catch (IllegalArgumentException expected) {
			// expected
		}
	}
	/** Verifies that simulation honours the waypoint time like the real update. */
	@Test
	public void simulationHonoursTheWaypointTimeLikeTheRealUpdate() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, 0.0, 1);
		Sequence sequence = sequenceOf(markers(0.0, 5.0,7.0), action);

		assertEquals("", sequence.updateActionsInSimulation(packetAt(HERE), 1.0));
		assertTrue(sequence.updateActionsInSimulation(packetAt(HERE), 5.0).contains("CountingAction"));
	}
	/** Verifies that simulation reports nothing when no action is ready. */
	@Test
	public void simulationReportsNothingWhenNoActionIsReady() {
		ArrayList<String> log = new ArrayList<>();
		Sequence sequence = sequenceOf(markers(0.0), CountingAction.at("A", log, 0));

		assertEquals("", sequence.updateActionsInSimulation(packetAt(new Pos2D(100, 100, 0)), 0.0));
	}
}
