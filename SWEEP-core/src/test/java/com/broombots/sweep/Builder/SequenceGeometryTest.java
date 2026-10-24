package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SequenceGeometryTest {
	/** Required agreement between the generated path and the ideal spline, in inches. */
	private static final double TOLERANCE = 1e-3;

	private static final double[][] CURVE = {{-30, -40}, {-10, -10}, {20, 0}, {40, 30}, {30, 50}};

	private static Sequence curvedSequence() {
		return PathTestSupport.build(new DefaultRobotMovementParameters(),
				-30, -40, 0, -10, -10, 20, 0, 40, 30, 30, 50, 90);
	}

	@Test
	public void everySampledPositionLiesOnTheIdealSpline() {
		Sequence sequence = curvedSequence();
		ArrayList<Pos2D> reference = PathTestSupport.referenceCurve(CURVE, 4000);

		for (PathPoint point : PathTestSupport.sample(sequence, 0.005)) {
			double error = PathTestSupport.distanceToPolyline(point.position, reference);
			assertTrue("Path left the spline by " + error + " in at " + point.position.x + "," + point.position.y,
					error <= TOLERANCE);
		}
	}

	@Test
	public void pathPassesThroughEveryWaypoint() {
		Sequence sequence = curvedSequence();
		ArrayList<PathPoint> samples = PathTestSupport.sample(sequence, 1e-5);

		for (double[] waypoint : CURVE) {
			Pos2D target = new Pos2D(waypoint[0], waypoint[1], 0);
			double closest = Double.MAX_VALUE;
			for (PathPoint point : samples) {
				closest = Math.min(closest, PathTestSupport.distance(point.position, target));
			}
			assertTrue("Missed waypoint " + waypoint[0] + "," + waypoint[1] + " by " + closest, closest <= TOLERANCE);
		}
	}

	@Test
	public void pathStartsAndEndsAtTheRequestedPose() {
		Sequence sequence = curvedSequence();

		PathPoint start = sequence.getMovement(0);
		assertEquals(-30, start.position.x, TOLERANCE);
		assertEquals(-40, start.position.y, TOLERANCE);
		assertEquals(0, start.position.angle, TOLERANCE);

		PathPoint end = sequence.getMovement(sequence.getLastTime());
		assertEquals(30, end.position.x, TOLERANCE);
		assertEquals(50, end.position.y, TOLERANCE);
		assertEquals(90, end.position.angle, TOLERANCE);
	}

	@Test
	public void collinearWaypointsProduceAStraightLine() {
		Sequence sequence = PathTestSupport.build(new DefaultRobotMovementParameters(),
				0, 10, 0, 24, 10, 48, 10, 72, 10, 0);

		double lastX = -1;
		for (PathPoint point : PathTestSupport.sample(sequence, 0.005)) {
			assertEquals(10, point.position.y, TOLERANCE);
			assertTrue("Path doubled back in x", point.position.x >= lastX - TOLERANCE);
			lastX = point.position.x;
		}
		assertEquals(72, lastX, TOLERANCE);
	}

	@Test
	public void diagonalLineStaysOnTheLineBetweenItsEndpoints() {
		Sequence sequence = PathTestSupport.build(new DefaultRobotMovementParameters(),
				0, 0, 0, 20, 20, 40, 40, 0);

		for (PathPoint point : PathTestSupport.sample(sequence, 0.005)) {
			assertEquals(point.position.x, point.position.y, TOLERANCE * Math.sqrt(2));
		}
	}

	@Test
	public void finerSampleRateDoesNotChangeTheGeometry() {
		Sequence fine = new SequenceBuilder(new DefaultRobotMovementParameters()).setSampleRate(0.0005)
				.start(-30, -40, 0).splineTo(-10, -10, 1).splineTo(20, 0, 1).splineTo(40, 30, 1).end(30, 50, 90)
				.build();
		ArrayList<Pos2D> reference = PathTestSupport.referenceCurve(CURVE, 4000);

		for (PathPoint point : PathTestSupport.sample(fine, 0.005)) {
			assertTrue(PathTestSupport.distanceToPolyline(point.position, reference) <= TOLERANCE);
		}
	}
}
