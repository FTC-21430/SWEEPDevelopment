package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class WaitWaypointTest {
	@Test
	public void constructorPreservesCoordinateAndDuration() {
		WaitWaypoint waypoint = new WaitWaypoint(7.0, 8.0, 9.0, 2.5);

		assertEquals(7.0, waypoint.getX(), 1e-9);
		assertEquals(8.0, waypoint.getY(), 1e-9);
		assertEquals(9.0, waypoint.getAngle(), 1e-9);
		assertEquals(2.5, waypoint.getDuration(), 1e-9);
		assertEquals(Waypoint.WaypointType.WAIT, waypoint.getType());
	}

	@Test
	public void coordinateConstructorRejectsNull() {
		try {
			new WaitWaypoint((Pos2D) null, 1.0);
		} catch (IllegalArgumentException expected) {
			return;
		}
		throw new AssertionError("Expected IllegalArgumentException");
	}
}
