package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Tests construction and heading/speed behavior of {@link SplineAngleWaypoint}.
 */
public class SplineAngleWaypointTest {
	/** Verifies that constructor preserves coordinate and speed. */
	@Test
	public void constructorPreservesCoordinateAndSpeed() {
		SplineAngleWaypoint waypoint = new SplineAngleWaypoint(3.0, 6.0, 45.0, 0.5);

		assertEquals(3.0, waypoint.getX(), 1e-9);
		assertEquals(6.0, waypoint.getY(), 1e-9);
		assertEquals(45.0, waypoint.getAngle(), 1e-9);
		assertEquals(0.5, waypoint.getSpeed(), 1e-9);
		assertEquals(Waypoint.WaypointType.SPLINE_ANGLE, waypoint.getType());
	}
	/** Verifies that coordinate constructor rejects null. */
	@Test
	public void coordinateConstructorRejectsNull() {
		try {
			new SplineAngleWaypoint((Pos2D) null, 1.0);
		} catch (IllegalArgumentException expected) {
			return;
		}
		throw new AssertionError("Expected IllegalArgumentException");
	}
}
