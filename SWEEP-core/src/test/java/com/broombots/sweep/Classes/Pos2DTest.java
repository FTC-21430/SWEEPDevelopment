package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class Pos2DTest {
	@Test
	public void magnitudeUsesHypotenuse() {
		Pos2D point = new Pos2D(3.0, 4.0, 30.0);

		assertEquals(5.0, point.getMagnitude(), 1e-9);
	}

	@Test
	public void distanceBetweenCoordinatesUsesHypotenuse() {
		Pos2D first = new Pos2D(0.0, 0.0, 0.0);
		Pos2D second = new Pos2D(6.0, 8.0, 0.0);

		assertEquals(10.0, Pos2D.getDistanceBetweenCoordinates(first, second), 1e-9);
	}

	@Test
	public void lerpClampsBelowZeroAndAboveOne() {
		Pos2D start = new Pos2D(0.0, 0.0, 0.0);
		Pos2D end = new Pos2D(10.0, 20.0, 30.0);

		Pos2D beforeStart = Pos2D.lerpPos2D(-1.0, start, end);
		Pos2D afterEnd = Pos2D.lerpPos2D(2.0, start, end);

		assertEquals(0.0, beforeStart.x, 1e-9);
		assertEquals(0.0, beforeStart.y, 1e-9);
		assertEquals(0.0, beforeStart.angle, 1e-9);
		assertEquals(10.0, afterEnd.x, 1e-9);
		assertEquals(20.0, afterEnd.y, 1e-9);
		assertEquals(30.0, afterEnd.angle, 1e-9);
	}

	@Test
	public void movementDirectionUsesAtan2() {
		Pos2D from = new Pos2D(0.0, 1.0, 0.0);
		Pos2D to = new Pos2D(0.0, 0.0, 0.0);

		assertEquals(Math.PI / 2.0, Pos2D.getMovementDirectionRad(from, to), 1e-9);
	}
}
