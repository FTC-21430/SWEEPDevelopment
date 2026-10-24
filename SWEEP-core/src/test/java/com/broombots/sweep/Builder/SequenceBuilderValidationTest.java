package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Robot;
import com.broombots.sweep.Classes.SWEEPAction;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class SequenceBuilderValidationTest {
	private static SequenceBuilder builder() {
		return new SequenceBuilder(new DefaultRobotMovementParameters()).setSampleRate(0.005);
	}

	private static void assertBuildFails(SequenceBuilder builder, String messagePart) {
		try {
			builder.build();
			fail("Expected build to fail");
		} catch (RuntimeException expected) {
			assertTrue("Unexpected message: " + expected.getMessage(),
					expected.getMessage().contains(messagePart));
		}
	}

	private static LocalizationPacket packetAt(double x, double y) {
		return new LocalizationPacket(x, y, 0, 0, 0);
	}

	private static class TestRobot extends Robot {
	}

	private static class UnplacedAction extends SWEEPAction {
		UnplacedAction() {
			super(new TestRobot());
		}

		@Override
		public void execute() {
		}

		@Override
		public void process() {
		}

		@Override
		public void end() {
		}
	}

	@Test
	public void buildWithoutStartFails() {
		assertBuildFails(builder().splineTo(20, 0, 1).end(40, 0, 0), "START");
	}

	@Test
	public void buildWithoutEndFails() {
		assertBuildFails(builder().start(0, 0, 0).splineTo(20, 0, 1), "END");
	}

	@Test
	public void buildWithOnlyOneWaypointFails() {
		assertBuildFails(builder().start(0, 0, 0), "two waypoints");
	}

	@Test
	public void buildWithNoWaypointsFails() {
		assertBuildFails(builder(), "two waypoints");
	}

	@Test
	public void buildWithoutMovementParametersFails() {
		assertBuildFails(new SequenceBuilder(null).start(0, 0, 0).end(10, 0, 0), "movement parameters");
	}

	@Test
	public void zeroAndNegativeSampleRatesAreRejected() {
		for (double rate : new double[]{0, -1, 1e-12}) {
			assertBuildFails(builder().setSampleRate(rate).start(0, 0, 0).end(10, 0, 0), "SampleRate");
		}
	}

	@Test
	public void addingANullActionIsRejected() {
		try {
			builder().addAction(null);
			fail("Expected NullPointerException");
		} catch (NullPointerException expected) {
			// expected
		}
	}

	@Test
	public void actionWithoutAPositionTriggersAtThePreviousWaypoint() {
		UnplacedAction action = new UnplacedAction();

		builder().start(0, 0, 0).splineTo(30, 20, 1).addAction(action);

		assertTrue(action.isPositionSet());
		assertTrue(action.checkTrigger(packetAt(30, 20), 0));
		assertFalse(action.checkTrigger(packetAt(0, 0), 0));
	}

	@Test
	public void actionAddedRightAfterStartTriggersAtTheStart() {
		UnplacedAction action = new UnplacedAction();

		builder().start(5, 6, 0).addAction(action);

		assertTrue(action.checkTrigger(packetAt(5, 6), 0));
	}

	@Test
	public void actionWithAnExplicitPositionKeepsIt() {
		UnplacedAction action = new UnplacedAction();
		action.setPosition(new Pos2D(-40, 40, 0));

		builder().start(0, 0, 0).splineTo(30, 20, 1).addAction(action);

		assertTrue(action.checkTrigger(packetAt(-40, 40), 0));
		assertFalse(action.checkTrigger(packetAt(30, 20), 0));
	}

	@Test
	public void outOfRangeSplineSpeedsBehaveLikeTheNearestValidSpeed() {
		double slow = builder().start(0, 0, 0).splineTo(20, 0, 0).end(40, 0, 0).build().getLastTime();
		double tooSlow = builder().start(0, 0, 0).splineTo(20, 0, -3).end(40, 0, 0).build().getLastTime();
		double fast = builder().start(0, 0, 0).splineTo(20, 0, 1).end(40, 0, 0).build().getLastTime();
		double tooFast = builder().start(0, 0, 0).splineTo(20, 0, 9).end(40, 0, 0).build().getLastTime();

		assertEquals(slow, tooSlow, 1e-9);
		assertEquals(fast, tooFast, 1e-9);
	}

	@Test
	public void waypointsBeyondTheFieldAreClippedToItsEdge() {
		Sequence sequence = builder().start(0, 0, 0).splineTo(500, 500, 1).end(40, 0, 0).build();
		double closestToEdge = Double.MAX_VALUE;

		for (PathPoint point : PathTestSupport.sample(sequence, 0.001)) {
			closestToEdge = Math.min(closestToEdge, PathTestSupport.distance(point.position, new Pos2D(72, 72, 0)));
			assertTrue("Path left the field to " + point.position.x + "," + point.position.y,
					Math.abs(point.position.x) < 100 && Math.abs(point.position.y) < 100);
		}
		assertTrue("Never reached the field corner, closest was " + closestToEdge, closestToEdge < 0.2);
	}

	@Test
	public void buildingTheSameBuilderTwiceGivesTheSamePath() {
		SequenceBuilder builder = builder().start(0, 0, 0).splineTo(20, 10, 1).waitAt(0.5).end(40, 0, 0);

		Sequence first = builder.build();
		Sequence second = builder.build();

		assertEquals(first.getLastTime(), second.getLastTime(), 1e-9);
		for (double t = 0; t <= first.getLastTime(); t += 0.05) {
			assertEquals(first.getPosition(t).x, second.getPosition(t).x, 1e-9);
			assertEquals(first.getPosition(t).y, second.getPosition(t).y, 1e-9);
		}
	}

	@Test
	public void pos2DAndPrimitiveOverloadsBuildTheSamePath() {
		Sequence primitive = builder().start(0, 0, 0).splineTo(20, 10, 1).end(40, 0, 0).build();
		Sequence objects = builder().start(new PathPoint() {
			{
				time = 0;
				position = new Pos2D(0, 0, 0);
				velocity = new Pos2D(0, 0, 0);
			}
		}).splineTo(new Pos2D(20, 10, 0), 1).end(new Pos2D(40, 0, 0)).build();

		assertEquals(primitive.getLastTime(), objects.getLastTime(), 1e-9);
	}

	@Test
	public void buildWithAStartTimeShiftsTheWholePath() {
		SequenceBuilder builder = builder().start(0, 0, 0).splineTo(20, 0, 1).end(40, 0, 0);

		double base = builder.build().getLastTime();
		double shifted = builder.build(5).getLastTime();

		assertEquals(base + 5, shifted, 1e-6);
	}
}
