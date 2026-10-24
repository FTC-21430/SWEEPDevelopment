package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.broombots.sweep.Splines.SplineWaypoint;
import com.broombots.sweep.Splines.Segments.FollowSplineSegment;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Tests for splineToOffset and splineToBackwards: same curve as splineTo, heading shifted by a constant. */
public class SequenceOffsetSplineTest {
	private static final double TOLERANCE = 1e-6;
	/** Verifies that builder. */
	private static SequenceBuilder builder() {
		return new SequenceBuilder(new DefaultRobotMovementParameters()).setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE)
				.start(0, 0, 0);
	}

	/** Smallest signed difference between two angles in degrees, in [-180, 180]. */
	private static double angleDifference(double a, double b) {
		double diff = (a - b) % 360;
		if (diff > 180) diff -= 360;
		if (diff < -180) diff += 360;
		return diff;
	}

	/** Samples that are past the first inch, where the heading has settled onto the curve. */
	private static ArrayList<PathPoint> settledSamples(Sequence sequence) {
		ArrayList<PathPoint> settled = new ArrayList<>();
		for (PathPoint point : PathTestSupport.sample(sequence, 0.01)) {
			if (PathTestSupport.distance(point.position, new Pos2D(0, 0, 0)) > 1) settled.add(point);
		}
		return settled;
	}

	/** Verifies adding whole turns to the requested offset does not change robot orientation. */
	@Test
	public void offsetsOutsideOneTurnKeepTheSameOrientationAndContinuousHeading() {
		Sequence ninety = builder().splineToOffset(20, 15, 90, 1).end(40, 0, 0).build();
		Sequence fourHundredFifty = builder().splineToOffset(20, 15, 450, 1).end(40, 0, 0).build();
		Sequence negativeTwoSeventy = builder().splineToOffset(20, 15, -270, 1).end(40, 0, 0).build();

		assertEquals(ninety.getLastTime(), fourHundredFifty.getLastTime(), TOLERANCE);
		assertEquals(ninety.getLastTime(), negativeTwoSeventy.getLastTime(), TOLERANCE);
		for (double time = 0.05; time < ninety.getLastTime() * 0.6; time += 0.02) {
			double expected = ninety.getPosition(time).angle;
			assertEquals(0, angleDifference(expected, fourHundredFifty.getPosition(time).angle), TOLERANCE);
			assertEquals(0, angleDifference(expected, negativeTwoSeventy.getPosition(time).angle), TOLERANCE);
		}
		int checked = 0;
		ArrayList<PathPoint> samples = PathTestSupport.sample(fourHundredFifty, 0.005);
		for (int i = 1; i < samples.size(); i++) {
			PathPoint previous = samples.get(i - 1);
			PathPoint current = samples.get(i);
			// Ignore the start-pose-to-path-tangent transition and the later explicit end-heading segment.
			if (previous.position.x < 1 || current.position.x > 19) continue;
			double change = Math.abs(current.position.angle - previous.position.angle);
			assertTrue("Whole-turn offset caused a heading jump of " + change + " degrees",
					change < 10);
			assertEquals(0, angleDifference(current.position.angle,
					negativeTwoSeventy.getPosition(current.time).angle), TOLERANCE);
			checked++;
		}
		assertTrue("Did not inspect enough samples on the offset spline: " + checked, checked > 50);
	}

	/** Verifies backwards heading stays continuous as curve direction crosses the atan2 +/-180-degree boundary. */
	@Test
	public void backwardsHeadingDoesNotJumpWhenTravelDirectionCrossesTheWrapBoundary() {
		Sequence sequence = builder()
				.splineToBackwards(-20, 10, 1)
				.splineToBackwards(-40, -10, 1)
				.end(-60, 0, 0)
				.build();

		int checked = 0;
		ArrayList<PathPoint> samples = PathTestSupport.sample(sequence, 0.005);
		for (int i = 1; i < samples.size(); i++) {
			PathPoint previous = samples.get(i - 1);
			PathPoint current = samples.get(i);
			if (current.position.x < -40) break;
			if (current.position.x > -1) continue;
			double change = Math.abs(current.position.angle - previous.position.angle);
			assertTrue("Backwards heading jumped by " + change + " degrees near x="
					+ current.position.x, change < 10);
			checked++;
		}

		assertTrue("Did not inspect enough samples around the +/-180-degree crossing: " + checked, checked > 100);
	}

	/** Verifies raw backwards spline headings cross the atan2 branch cut while remaining physically equivalent. */
	@Test
	public void backwardsSplineRawHeadingCrossesAtan2BranchCut() {
		FollowSplineSegment segment = new FollowSplineSegment(
				new SplineWaypoint(0, 0, 1),
				new SplineWaypoint(-1, 0.1, 1),
				new SplineWaypoint(-2, -0.1, 1),
				new SplineWaypoint(-3, 0.3, 1),
				180);

		double startHeading = segment.getPosition(0).angle;
		double endHeading = segment.getPosition(1).angle;

		assertTrue("Test curve did not cross the numeric angle branch cut",
				Math.abs(endHeading - startHeading) > 350);
		assertTrue("The curve's physical orientation should change by only a few degrees",
				Math.abs(angleDifference(startHeading, endHeading)) < 10);
	}

	/** Verifies the backwards offset remains 180 degrees from travel direction on both sides of the wrap boundary. */
	@Test
	public void backwardsOrientationRemainsOppositeTravelAcrossTheWrapBoundary() {
		Sequence forwards = builder()
				.splineTo(-20, 10, 1)
				.splineTo(-40, -10, 1)
				.end(-60, 0, 0)
				.build();
		Sequence backwards = builder()
				.splineToBackwards(-20, 10, 1)
				.splineToBackwards(-40, -10, 1)
				.end(-60, 0, 0)
				.build();

		int checked = 0;
		for (PathPoint point : PathTestSupport.sample(backwards, 0.01)) {
			if (point.position.x < -40) break;
			if (point.position.x > -1) continue;
			double forwardHeading = forwards.getPosition(point.time).angle;
			assertEquals(180, Math.abs(angleDifference(point.position.angle, forwardHeading)), 1e-3);
			checked++;
		}

		assertTrue("Did not compare enough backwards spline samples: " + checked, checked > 50);
	}

	/** Verifies that offset of zero matches spline to. */
	@Test
	public void offsetOfZeroMatchesSplineTo() {
		Sequence plain = builder().splineTo(20, 15, 1).end(40, 0, 0).build();
		Sequence offset = builder().splineToOffset(20, 15, 0, 1).end(40, 0, 0).build();

		assertEquals(plain.getLastTime(), offset.getLastTime(), TOLERANCE);
		for (double t = 0; t <= plain.getLastTime(); t += 0.05) {
			assertEquals(plain.getPosition(t).x, offset.getPosition(t).x, TOLERANCE);
			assertEquals(plain.getPosition(t).y, offset.getPosition(t).y, TOLERANCE);
			assertEquals(0, angleDifference(plain.getPosition(t).angle, offset.getPosition(t).angle), TOLERANCE);
		}
	}
	/** Verifies that backwards follows the same curve as forwards. */
	@Test
	public void backwardsFollowsTheSameCurveAsForwards() {
		Sequence plain = builder().splineTo(20, 15, 1).end(40, 0, 0).build();
		Sequence backwards = builder().splineToBackwards(20, 15, 1).end(40, 0, 0).build();

		assertEquals(plain.getLastTime(), backwards.getLastTime(), TOLERANCE);
		for (double t = 0; t <= plain.getLastTime(); t += 0.05) {
			assertEquals(plain.getPosition(t).x, backwards.getPosition(t).x, TOLERANCE);
			assertEquals(plain.getPosition(t).y, backwards.getPosition(t).y, TOLERANCE);
		}
	}
	/** Verifies that backwards heading is opposite the direction of travel. */
	@Test
	public void backwardsHeadingIsOppositeTheDirectionOfTravel() {
		Sequence plain = builder().splineTo(20, 15, 1).splineTo(40, 15, 1).end(60, 0, 0).build();
		Sequence backwards = builder().splineToBackwards(20, 15, 1).splineToBackwards(40, 15, 1).end(60, 0, 0).build();

		int checked = 0;
		for (PathPoint point : settledSamples(backwards)) {
			// the ending segment is a normal spline, so only compare while the robot is still on the backwards part
			if (point.position.x > 38) break;
			double forwardHeading = plain.getPosition(point.time).angle;
			assertEquals(180, Math.abs(angleDifference(point.position.angle, forwardHeading)), 1e-3);
			checked++;
		}
		assertTrue("too few samples were compared: " + checked, checked > 50);
	}
	/** Verifies that arbitrary offset is added to the direction of travel. */
	@Test
	public void arbitraryOffsetIsAddedToTheDirectionOfTravel() {
		Sequence plain = builder().splineTo(20, 15, 1).end(40, 0, 0).build();
		Sequence offset = builder().splineToOffset(20, 15, 90, 1).end(40, 0, 0).build();

		int checked = 0;
		for (PathPoint point : settledSamples(offset)) {
			if (point.position.x > 19) break;
			assertEquals(90, angleDifference(point.position.angle, plain.getPosition(point.time).angle), 1e-3);
			checked++;
		}
		assertTrue("too few samples were compared: " + checked, checked > 20);
	}
	/** Verifies that pos2 doverload matches the primitive overload and ignores its angle. */
	@Test
	public void pos2DOverloadMatchesThePrimitiveOverloadAndIgnoresItsAngle() {
		Sequence primitive = builder().splineToOffset(20, 15, 180, 1).end(40, 0, 0).build();
		Sequence object = builder().splineToOffset(new Pos2D(20, 15, 77), 180, 1).end(40, 0, 0).build();

		assertEquals(primitive.getLastTime(), object.getLastTime(), TOLERANCE);
		assertEquals(primitive.getPosition(1).angle, object.getPosition(1).angle, TOLERANCE);
	}
	/** Verifies that out of range speed behaves like the nearest valid speed. */
	@Test
	public void outOfRangeSpeedBehavesLikeTheNearestValidSpeed() {
		double fast = builder().splineToBackwards(20, 0, 1).end(40, 0, 0).build().getLastTime();
		double tooFast = builder().splineToBackwards(20, 0, 9).end(40, 0, 0).build().getLastTime();
		double slow = builder().splineToBackwards(20, 0, 0).end(40, 0, 0).build().getLastTime();
		double tooSlow = builder().splineToBackwards(20, 0, -3).end(40, 0, 0).build().getLastTime();

		assertEquals(fast, tooFast, 1e-9);
		assertEquals(slow, tooSlow, 1e-9);
	}
	/** Verifies that waypoints beyond the field are clipped. */
	@Test
	public void waypointsBeyondTheFieldAreClipped() {
		Sequence sequence = builder().splineToBackwards(500, 500, 1).end(40, 0, 0).build();

		for (PathPoint point : PathTestSupport.sample(sequence, 0.01)) {
			assertTrue(Math.abs(point.position.x) < 100 && Math.abs(point.position.y) < 100);
		}
	}
	/** Verifies that robot holds the backwards heading while waiting at the waypoint. */
	@Test
	public void robotHoldsTheBackwardsHeadingWhileWaitingAtTheWaypoint() {
		Sequence sequence = builder().splineToBackwards(30, 0, 1).waitAt(1).end(60, 0, 0).build();
		Pos2D waypoint = new Pos2D(30, 0, 0);
		int waiting = 0;

		for (PathPoint point : PathTestSupport.sample(sequence, 0.01)) {
			boolean stopped = PathTestSupport.distance(point.position, waypoint) < 1e-3 && PathTestSupport.speed(point) < 1e-6;
			if (!stopped) continue;
			assertEquals(180, Math.abs(angleDifference(point.position.angle, 0)), 1e-3);
			waiting++;
		}
		assertTrue("never found the robot waiting at the waypoint", waiting > 50);
	}
	/** Verifies that mixed forward and backwards route builds and ends at the requested pose. */
	@Test
	public void mixedForwardAndBackwardsRouteBuildsAndEndsAtTheRequestedPose() {
		Sequence sequence = builder().splineTo(20, 10, 1).addBreak().splineToBackwards(0, 30, 1).addBreak()
				.splineToOffset(-20, 10, 90, 1).end(-30, 0, 45).build();

		PathPoint last = sequence.getMovement(sequence.getLastTime());
		assertEquals(-30, last.position.x, 1e-3);
		assertEquals(0, last.position.y, 1e-3);
		assertEquals(0, angleDifference(45, last.position.angle), 1e-3);
		assertEquals(0, PathTestSupport.speed(last), 1e-6);
	}
}
