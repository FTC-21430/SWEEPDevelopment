package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/** Tests coordinate construction, default heading, and distance calculations. */
public class CoordinateTest {
	/** Verifies that constructor preserves values. */
	@Test
	public void constructorPreservesValues() {
		Coordinate coordinate = new Coordinate(12.5, -4.0, 90.0);

		assertEquals(12.5, coordinate.getX(), 1e-9);
		assertEquals(-4.0, coordinate.getY(), 1e-9);
		assertEquals(90.0, coordinate.getAngle(), 1e-9);
	}
	/** Verifies that two-argument constructor defaults angle to zero. */
	@Test
	public void twoArgConstructorDefaultsAngleToZero() {
		Coordinate coordinate = new Coordinate(8.0, 3.0);

		assertEquals(8.0, coordinate.getX(), 1e-9);
		assertEquals(3.0, coordinate.getY(), 1e-9);
		assertEquals(0.0, coordinate.getAngle(), 1e-9);
	}
	/** Verifies that distance between coordinates uses Euclidean distance. */
	@Test
	public void distanceBetweenCoordinatesUsesEuclideanDistance() {
		Coordinate first = new Coordinate(0.0, 0.0, 0.0);
		Coordinate second = new Coordinate(3.0, 4.0, 180.0);

		assertEquals(5.0, Coordinate.getDistanceBetweenCoordinates(first, second), 1e-9);
	}
}
