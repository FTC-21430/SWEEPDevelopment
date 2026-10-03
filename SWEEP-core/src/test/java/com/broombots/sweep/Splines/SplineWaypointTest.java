package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SplineWaypointTest {
	@Test
	public void constructorPreservesCoordinateAndSpeed() {
		SplineWaypoint waypoint = new SplineWaypoint(2.0, 4.0, 0.75);

		assertEquals(2.0, waypoint.getX(), 1e-9);
		assertEquals(4.0, waypoint.getY(), 1e-9);
		assertEquals(0.0, waypoint.getAngle(), 1e-9);
		assertEquals(0.75, waypoint.getSpeed(), 1e-9);
		assertEquals(Waypoint.WaypointType.SPLINE, waypoint.getType());
	}

	@Test
	public void coordinateConstructorRejectsNull() {
		try {
			new SplineWaypoint((Pos2D) null, 1.0);
		} catch (IllegalArgumentException expected) {
			return;
		}
		throw new AssertionError("Expected IllegalArgumentException");
	}
}
