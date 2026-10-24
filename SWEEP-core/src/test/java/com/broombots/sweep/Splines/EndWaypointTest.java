package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Tests construction, terminal speed, and type of {@link EndWaypoint}.
 */
public class EndWaypointTest {
	/** Verifies that constructor preserves coordinate and speed. */
	@Test
	public void constructorPreservesCoordinateAndSpeed() {
		EndWaypoint waypoint = new EndWaypoint(4.0, 5.0, 6.0);

		assertEquals(4.0, waypoint.getX(), 1e-9);
		assertEquals(5.0, waypoint.getY(), 1e-9);
		assertEquals(6.0, waypoint.getAngle(), 1e-9);
		assertEquals(1.0, waypoint.getSpeed(), 1e-9);
		assertEquals(Waypoint.WaypointType.END, waypoint.getType());
	}
	/** Verifies that coordinate constructor rejects null. */
	@Test
	public void coordinateConstructorRejectsNull() {
		try {
			new EndWaypoint((Pos2D) null);
		} catch (IllegalArgumentException expected) {
			return;
		}
		throw new AssertionError("Expected IllegalArgumentException");
	}
}
