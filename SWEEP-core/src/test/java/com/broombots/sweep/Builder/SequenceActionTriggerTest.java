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
 * Checks that actions fire alongside their intended waypoint despite waits and repeated trigger positions.
 * Tying actions to path progress prevents work from firing at the wrong visit as the robot follows a route.
 */
public class SequenceActionTriggerTest {
	/** Simulated action-update interval, in seconds. */
	private static final double TIME_STEP = 0.005; // seconds between simulated control loops
	/** Trigger-position radius, in inches. */
	private static final double TOLERANCE = 1.0; // inches
	/** Position visited twice by the route, in inches and degrees. */
	private static final Pos2D SHARED_POSITION = new Pos2D(24, 0, 0);

	/**
	 * Creates localization data at the supplied robot pose.
	 * @param position robot position in inches and heading in degrees
	 * @return localization packet with zero velocity
	 */
	private static LocalizationPacket packetAt(Pos2D position) {
		return new LocalizationPacket(position.x, position.y, position.angle, 0, 0);
	}

	/**
	 * Creates a mutable list of waypoint-time markers.
	 * @param times waypoint times in seconds
	 * @return marker list
	 */
	private static ArrayList<Double> markers(Double... times) {
		return new ArrayList<>(Arrays.asList(times));
	}

	/** Records trigger execution count and the simulated path time of execution. */
	private static class RecordingAction extends SWEEPAction {
		final Clock clock;
		double executedAt = Double.NaN;
		int executeCount;

		/**
		 * Creates an action tied to the supplied trigger position and waypoint.
		 * @param clock simulated path clock
		 * @param position trigger position in inches and heading in degrees, or null for an unplaced action
		 * @param splineIndex waypoint marker index, or -1 for position-only triggering
		 */
		RecordingAction(Clock clock, Pos2D position, int splineIndex) {
			super(new TestRobot(), position, 0.0, TOLERANCE, 0.0);
			this.clock = clock;
			setSplineIndex(splineIndex);
		}

		/** @return whether the action has executed at least once */
		boolean hasExecuted() {
			return executeCount > 0;
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
			executeCount++;
			executedAt = clock.now;
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
		}
	}

	/** Mutable path clock observed by recording actions; time is in seconds. */
	private static class Clock {
		double now;
	}

	/** Minimal robot instance for action tests without hardware dependencies. */
	private static class TestRobot extends Robot {
	}
	/** Verifies that action is not triggered before its waypoint time even at the trigger position. */
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
	/** Verifies that action triggers once its waypoint time is reached at the trigger position. */
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
	/** Verifies that action past its waypoint time still needs the robot at the trigger position. */
	@Test
	public void actionPastItsWaypointTimeStillNeedsTheRobotAtTheTriggerPosition() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 3);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 2.0, 5.0, 8.0));

		sequence.updateActions(packetAt(new Pos2D(100, 100, 0)), 6.0);

		assertEquals(0, action.executeCount);
	}
	/** Verifies that action triggers within tolerance of its position at its waypoint time. */
	@Test
	public void actionTriggersWithinToleranceOfItsPositionAtItsWaypointTime() {
		Clock clock = new Clock();
		RecordingAction action = new RecordingAction(clock, SHARED_POSITION, 1);
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action},
				markers(0.0, 2.0, 4.0));

		sequence.updateActions(packetAt(new Pos2D(24 + TOLERANCE * 0.9, 0, 0)), 2.0);

		assertEquals(1, action.executeCount);
	}
	/** Verifies that actions are held by their own waypoint time and fire in order. */
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
	/** Verifies that action waiting on awaypoint blocks later actions in the queue. */
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
	/** Verifies that wait time and waypoint time both have to be satisfied. */
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
	/** Verifies that action without aspline index still triggers by position. */
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
	 * Builds a route that visits the shared position twice, with a wait at each visit.
	 * A purely position-based trigger could otherwise fire on the wrong visit.
	 * @param first action attached to the first visit
	 * @param second action attached to the second visit
	 * @return builder for the two-visit route
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

	/**
	 * Drives the sequence the way the interpreter does, following the planned position exactly.
	 * @param sequence sequence whose actions are simulated
	 * @param clock clock updated on every simulation step, in seconds
	 */
	private static void simulate(Sequence sequence, Clock clock) {
		for (double t = 0; t <= sequence.getLastTime(); t += TIME_STEP) {
			clock.now = t;
			sequence.updateActions(packetAt(sequence.getPosition(t)), t);
		}
	}

	/**
	 * Finds path-time intervals during which the planned position is within trigger tolerance.
	 * @param sequence planned route to inspect
	 * @return seconds at which the path enters and exits the shared-position radius, as [enter, exit] pairs
	 */
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

	/** Verifies that visits are distinct, rather than one continuous tolerance window. */
	@Test
	public void routeRevisitsTheSharedPositionSoTheScenarioIsMeaningful() {
		Clock clock = new Clock();
		Sequence sequence = revisitingRoute(new RecordingAction(clock, null, -1),
				new RecordingAction(clock, null, -1)).build();

		assertTrue("Expected two separate visits", visitsToSharedPosition(sequence).size() >= 2);
	}

	/** Ensures waypoint markers distinguish the second visit from an earlier identical trigger position. */
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
	/** Verifies that action after the first wait still triggers at the first visit when queued behind nothing. */
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
