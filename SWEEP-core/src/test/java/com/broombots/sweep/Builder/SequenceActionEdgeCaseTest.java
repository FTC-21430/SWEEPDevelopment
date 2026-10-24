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

/** Unusual action queues and timing combinations. */
public class SequenceActionEdgeCaseTest {
	private static final Pos2D HERE = new Pos2D(10, 10, 0);

	private static class TestRobot extends Robot {
	}

	private static class CountingAction extends SWEEPAction {
		final String name;
		final ArrayList<String> log;
		int executeCount;
		int processCount;
		int endCount;

		CountingAction(String name, ArrayList<String> log, Pos2D position, double holdTime, double tolerance,
					   double waitTime, int splineIndex) {
			super(new TestRobot(), position, holdTime, tolerance, waitTime);
			this.name = name;
			this.log = log;
			setSplineIndex(splineIndex);
		}

		static CountingAction at(String name, ArrayList<String> log, double holdTime) {
			return new CountingAction(name, log, HERE, holdTime, 1.0, 0.0, -1);
		}

		@Override
		public void execute() {
			executeCount++;
			log.add(name + ".execute");
		}

		@Override
		public void process() {
			processCount++;
		}

		@Override
		public void end() {
			endCount++;
			log.add(name + ".end");
		}
	}

	private static Sequence sequenceOf(ArrayList<Double> markers, SWEEPAction... actions) {
		return new Sequence(new ArrayList<PathPoint>(), actions, markers);
	}

	private static ArrayList<Double> markers(Double... times) {
		return new ArrayList<>(Arrays.asList(times));
	}

	private static LocalizationPacket packetAt(Pos2D position) {
		return new LocalizationPacket(position.x, position.y, position.angle, 0, 0);
	}

	private static void update(Sequence sequence, Pos2D position, double time, int times) {
		for (int i = 0; i < times; i++) {
			sequence.updateActions(packetAt(position), time);
		}
	}

	@Test
	public void updatingWithNoActionsIsHarmless() {
		Sequence sequence = sequenceOf(markers(0.0));

		update(sequence, HERE, 0, 100);
	}

	@Test
	public void completedActionIsNeverTriggeredAgain() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = CountingAction.at("A", log, 0);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 0, 20);

		assertEquals(1, action.executeCount);
		assertEquals(1, action.endCount);
	}

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

	@Test
	public void actionEndsBeforeTheNextOneStarts() {
		ArrayList<String> log = new ArrayList<>();
		Sequence sequence = sequenceOf(markers(0.0), CountingAction.at("A", log, 0), CountingAction.at("B", log, 0));

		update(sequence, HERE, 0, 10);

		assertTrue(log.indexOf("A.end") < log.indexOf("B.execute"));
	}

	@Test
	public void triggerPositionWithNaNNeverTriggers() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, new Pos2D(Double.NaN, 0, 0), 0, 1.0, 0, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 0, 20);

		assertEquals(0, action.executeCount);
	}

	@Test
	public void infiniteWaitTimeNeverTriggers() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, Double.POSITIVE_INFINITY, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 1e9, 5);

		assertEquals(0, action.executeCount);
	}

	@Test
	public void negativeWaitTimeTriggersImmediately() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, -5.0, -1);
		Sequence sequence = sequenceOf(markers(0.0), action);

		update(sequence, HERE, 0, 1);

		assertEquals(1, action.executeCount);
	}

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

	@Test
	public void actionsWithoutAnyMarkersStillTrigger() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, 0.0, 3);
		Sequence sequence = sequenceOf(markers(), action);

		update(sequence, HERE, 0, 1);

		assertEquals(1, action.executeCount);
	}

	@Test
	public void nullMarkersAreRejectedLikeTheOtherArguments() {
		try {
			new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[0], null);
			fail("Expected IllegalArgumentException");
		} catch (IllegalArgumentException expected) {
			// expected
		}
	}

	@Test
	public void simulationHonoursTheWaypointTimeLikeTheRealUpdate() {
		ArrayList<String> log = new ArrayList<>();
		CountingAction action = new CountingAction("A", log, HERE, 0, 1.0, 0.0, 1);
		Sequence sequence = sequenceOf(markers(0.0, 5.0,7.0), action);

		assertEquals("", sequence.updateActionsInSimulation(packetAt(HERE), 1.0));
		assertTrue(sequence.updateActionsInSimulation(packetAt(HERE), 5.0).contains("CountingAction"));
	}

	@Test
	public void simulationReportsNothingWhenNoActionIsReady() {
		ArrayList<String> log = new ArrayList<>();
		Sequence sequence = sequenceOf(markers(0.0), CountingAction.at("A", log, 0));

		assertEquals("", sequence.updateActionsInSimulation(packetAt(new Pos2D(100, 100, 0)), 0.0));
	}
}
