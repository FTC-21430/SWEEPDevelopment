package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPAction;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;

/** Sampling of hand built paths, so interpolation is checked without the path planner involved. */
public class SequenceInterpolationTest {
	private static final double EPS = 1e-9;

	private static PathPoint point(double time, double x, double y, double angle, double vx, double vy) {
		PathPoint point = new PathPoint();
		point.time = time;
		point.position = new Pos2D(x, y, angle);
		point.velocity = new Pos2D(vx, vy, 0);
		return point;
	}

	private static Sequence sequenceOf(PathPoint... points) {
		ArrayList<PathPoint> path = new ArrayList<>();
		for (PathPoint point : points) path.add(point);
		return new Sequence(path, new SWEEPAction[0], new ArrayList<Double>());
	}

	@Test
	public void midpointBlendsPositionVelocityAndTime() {
		Sequence sequence = sequenceOf(point(0, 0, 0, 0, 0, 0), point(2, 10, 20, 90, 4, 8));

		PathPoint middle = sequence.getMovement(1);

		assertEquals(5, middle.position.x, EPS);
		assertEquals(10, middle.position.y, EPS);
		assertEquals(45, middle.position.angle, EPS);
		assertEquals(2, middle.velocity.x, EPS);
		assertEquals(4, middle.velocity.y, EPS);
		assertEquals(1, middle.time, EPS);
	}

	@Test
	public void sampleExactlyOnAPointReturnsThatPoint() {
		Sequence sequence = sequenceOf(point(0, 0, 0, 0, 0, 0), point(1, 10, 0, 0, 1, 0), point(3, 30, 0, 0, 2, 0));

		assertEquals(10, sequence.getPosition(1).x, EPS);
		assertEquals(30, sequence.getPosition(3).x, EPS);
	}

	@Test
	public void timesBeforeTheStartClampToTheFirstPoint() {
		Sequence sequence = sequenceOf(point(0, 1, 2, 3, 0, 0), point(2, 10, 20, 30, 4, 8));

		PathPoint before = sequence.getMovement(-100);

		assertEquals(1, before.position.x, EPS);
		assertEquals(2, before.position.y, EPS);
		assertEquals(3, before.position.angle, EPS);
	}

	@Test
	public void timesAfterTheEndClampToTheLastPoint() {
		Sequence sequence = sequenceOf(point(0, 0, 0, 0, 0, 0), point(1, 5, 5, 5, 1, 1), point(2, 10, 20, 30, 4, 8));

		PathPoint after = sequence.getMovement(100);

		assertEquals(10, after.position.x, EPS);
		assertEquals(20, after.position.y, EPS);
		assertEquals(30, after.position.angle, EPS);
	}

	@Test
	public void unevenlySpacedPointsInterpolateInsideTheirOwnInterval() {
		Sequence sequence = sequenceOf(point(0, 0, 0, 0, 0, 0), point(0.1, 1, 0, 0, 0, 0), point(10, 100, 0, 0, 0, 0));

		assertEquals(0.5, sequence.getPosition(0.05).x, EPS);
		assertEquals(50.5, sequence.getPosition(5.05).x, 1e-6);
	}

	@Test
	public void lastTimeIsTheTimeOfTheFinalPoint() {
		Sequence sequence = sequenceOf(point(0, 0, 0, 0, 0, 0), point(7.25, 5, 0, 0, 0, 0));

		assertEquals(7.25, sequence.getLastTime(), EPS);
	}

	@Test
	public void getPositionMatchesGetMovementEverywhere() {
		Sequence sequence = sequenceOf(point(0, 0, 0, 0, 0, 0), point(1, 4, 3, 10, 0, 0), point(2.5, -2, 8, 20, 0, 0));

		for (double t = -1; t < 4; t += 0.07) {
			assertEquals(sequence.getMovement(t).position.x, sequence.getPosition(t).x, EPS);
			assertEquals(sequence.getMovement(t).position.y, sequence.getPosition(t).y, EPS);
		}
	}

	@Test
	public void largePathIsSearchedCorrectly() {
		PathPoint[] points = new PathPoint[2001];
		for (int i = 0; i < points.length; i++) {
			points[i] = point(i * 0.01, i * 0.1, 0, 0, 10, 0);
		}
		Sequence sequence = sequenceOf(points);

		for (double t = 0; t <= 20; t += 0.0137) {
			assertEquals(10 * t, sequence.getPosition(t).x, 1e-6);
		}
	}
}
