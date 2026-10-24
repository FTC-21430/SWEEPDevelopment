package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Robot;
import com.broombots.sweep.Classes.SWEEPAction;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Checks that actions fire alongside the waypoint they were defined after, even when
 * wait segments exist and other parts of the route share the same trigger position.
 */
public class SequenceActionTriggerTest {
	private static final double TIME_STEP = 0.005; // seconds between simulated control loops
	private static final double TOLERANCE = 1.0; // inches
	private static final Pos2D SHARED_POSITION = new Pos2D(24, 0, 0);

	private static LocalizationPacket packetAt(Pos2D position) {
		return new LocalizationPacket(position.x, position.y, position.angle, 0, 0);
	}

	private static ArrayList<Double> markers(Double... times) {
		return new ArrayList<>(Arrays.asList(times));
	}

	/** Records the path time at which it was executed. */
	private static class RecordingAction extends SWEEPAction {
		final Clock clock;
		double executedAt = Double.NaN;
		int executeCount;

		RecordingAction(Clock clock, Pos2D position, int splineIndex) {
			super(new TestRobot(), position, 0.0, TOLERANCE, 0.0);
			this.clock = clock;
			setSplineIndex(splineIndex);
		}

		boolean hasExecuted() {
			return executeCount > 0;
		}

		@Override
		public void execute() {
			executeCount++;
			executedAt = clock.now;
		}

		@Override
		public void process() {
		}

		@Override
		public void end() {
		}
	}

	private static class Clock {
		double now;
	}

	private static class TestRobot extends Robot {
	}

	@Test
	public void actionIsNotTriggeredBeforeItsWaypointTimeEvenAtTheTriggerPosition() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 3);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 2.0, 5.0, 8.0));

		for (double t : new double[]{0.0, 1.0, 2.0, 4.99}) {
			clock.now = t;
			sequence.updateActions(packetAt(SHARED_POSITION), t);
		}

		assertEquals(0, action.executeCount);
	}

	@Test
	public void actionTriggersOnceItsWaypointTimeIsReachedAtTheTriggerPosition() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 3);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 2.0, 5.0, 8.0));

		clock.now = 5.0;
		sequence.updateActions(packetAt(SHARED_POSITION), 5.0);

		assertEquals(1, action.executeCount);
		assertEquals(5.0, action.executedAt, 1e-9);
	}

	@Test
	public void actionPastItsWaypointTimeStillNeedsTheRobotAtTheTriggerPosition() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 3);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 2.0, 5.0, 8.0));

		sequence.updateActions(packetAt(new Pos2D(100, 100, 0)), 6.0);

		assertEquals(0, action.executeCount);
	}

	@Test
	public void actionTriggersWithinToleranceOfItsPositionAtItsWaypointTime() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 1);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 4.0));

		sequence.updateActions(packetAt(new Pos2D(24 + TOLERANCE * 0.9, 0, 0)), 2.0);

		assertEquals(1, action.executeCount);
	}

	@Test
	public void actionsAreHeldByTheirOwnWaypointTimeAndFireInOrder() {
		Clock clock = new Clock();
		RecordingAction early = new RecordingAction(clock, SHARED_POSITION, 1);
		RecordingAction late = new RecordingAction(clock, SHARED_POSITION, 3);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{early, late},
				markers(0.0, 2.0, 2.0, 6.0, 9.0));

		for (double t = 0; t <= 9.0; t += 0.5) {
			clock.now = t;
			sequence.updateActions(packetAt(SHARED_POSITION), t);
		}

		assertTrue(early.executedAt >= 2.0 && early.executedAt < 6.0);
		assertTrue("Late action fired at " + late.executedAt, late.executedAt >= 6.0);
		assertTrue(early.executedAt < late.executedAt);
	}

	@Test
	public void actionWaitingOnAWaypointBlocksLaterActionsInTheQueue() {
		Clock clock = new Clock();
		RecordingAction first = new RecordingAction(clock, SHARED_POSITION, 3);
		RecordingAction second = new RecordingAction(clock, new Pos2D(60, 60, 0), 1);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{first, second},
				markers(0.0, 2.0, 2.0, 6.0, 9.0));

		sequence.updateActions(packetAt(new Pos2D(60, 60, 0)), 3.0);

		assertEquals("Actions must run in the order they were added", 0, second.executeCount);
	}

	@Test
	public void waitTimeAndWaypointTimeBothHaveToBeSatisfied() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 1);
		action.setWaitTime(7.0);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 4.0));

		sequence.updateActions(packetAt(SHARED_POSITION), 5.0);
		assertEquals(0, action.executeCount);

		sequence.updateActions(packetAt(SHARED_POSITION), 7.0);
		assertEquals(1, action.executeCount);
	}

	@Test
	public void actionWithoutASplineIndexStillTriggersByPosition() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, -1);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 4.0));

		sequence.updateActions(packetAt(SHARED_POSITION), 0.0);

		assertEquals(1, action.executeCount);
	}

	/**
	 * Route that passes the shared position twice, with a wait at each visit, so a purely position based
	 * trigger could fire on the wrong visit.
	 */
	private static SequenceBuilder revisitingRoute(RecordingAction first, RecordingAction second) {
		return new SequenceBuilder(new DefaultRobotMovementParameters())
				.setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE)
				.start(0, 0, 0)
				.splineTo(24, 0, 1)
				.waitAt(1.0)
				.addAction(first)
				.splineTo(24, 40, 1)
				.splineTo(0, 40, 1)
				.splineTo(24, 0, 1)
				.waitAt(1.0)
				.addAction(second)
				.splineTo(48, 0, 1)
				.end(72, 0, 0);
	}

	/** Drives the sequence the way the interpreter does, following the planned position exactly. */
	private static void simulate(Sequence sequence, Clock clock) {
		for (double t = 0; t <= sequence.getLastTime(); t += TIME_STEP) {
			clock.now = t;
			sequence.updateActions(packetAt(sequence.getPosition(t)), t);
		}
	}

	/** Seconds at which the planned path is inside the tolerance of the shared position, as [enter, exit] pairs. */
	private static ArrayList<double[]> visitsToSharedPosition(Sequence sequence) {
		ArrayList<double[]> visits = new ArrayList<>();
		double enter = Double.NaN;
		for (double t = 0; t <= sequence.getLastTime(); t += TIME_STEP) {
			boolean inside = PathTestSupport.distance(sequence.getPosition(t), SHARED_POSITION) <= TOLERANCE;
			if (inside && Double.isNaN(enter)) enter = t;
			if (!inside && !Double.isNaN(enter)) {
				visits.add(new double[]{enter, t});
				enter = Double.NaN;
			}
		}
		if (!Double.isNaN(enter)) visits.add(new double[]{enter, sequence.getLastTime()});
		return visits;
	}

	@Test
	public void routeRevisitsTheSharedPositionSoTheScenarioIsMeaningful() {
		Clock clock = new Clock();
		Sequence sequence = revisitingRoute(new RecordingAction(clock, null, -1),
				new RecordingAction(clock, null, -1)).build();

		assertTrue("Expected two separate visits", visitsToSharedPosition(sequence).size() >= 2);
	}

	@Test
	public void actionAfterTheSecondWaitTriggersOnTheSecondVisitNotTheFirst() {
		Clock clock = new Clock();
		RecordingAction first = new RecordingAction(clock, null, -1);
		RecordingAction second = new RecordingAction(clock, null, -1);
		Sequence sequence = revisitingRoute(first, second).build();
		ArrayList<double[]> visits = visitsToSharedPosition(sequence);

		simulate(sequence, clock);

		assertTrue("First action never ran", first.hasExecuted());
		assertTrue("Second action never ran", second.hasExecuted());
		double[] firstVisit = visits.get(0);
		double[] secondVisit = visits.get(1);
		assertTrue("First action fired at " + first.executedAt + " outside the first visit",
				first.executedAt >= firstVisit[0] && first.executedAt <= firstVisit[1]);
		assertTrue("Second action fired at " + second.executedAt + " but the second visit began at "
				+ secondVisit[0], second.executedAt >= secondVisit[0] && second.executedAt <= secondVisit[1]);
	}

	@Test
	public void actionAfterTheFirstWaitStillTriggersAtTheFirstVisitWhenQueuedBehindNothing() {
		Clock clock = new Clock();
		RecordingAction first = new RecordingAction(clock, null, -1);
		RecordingAction second = new RecordingAction(clock, null, -1);
		Sequence sequence = revisitingRoute(first, second).build();
		ArrayList<double[]> visits = visitsToSharedPosition(sequence);

		simulate(sequence, clock);

		assertTrue(first.executedAt < visits.get(1)[0]);
	}
}
