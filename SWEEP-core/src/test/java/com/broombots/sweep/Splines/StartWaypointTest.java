package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class StartWaypointTest {
	@Test
	public void constructorPreservesCoordinate() {
		StartWaypoint waypoint = new StartWaypoint(1.0, 2.0, 3.0);

		assertEquals(1.0, waypoint.getX(), 1e-9);
		assertEquals(2.0, waypoint.getY(), 1e-9);
		assertEquals(3.0, waypoint.getAngle(), 1e-9);
		assertEquals(Waypoint.WaypointType.START, waypoint.getType());
	}

	@Test
	public void coordinateConstructorRejectsNull() {
		try {
			new StartWaypoint((Pos2D) null);
		} catch (IllegalArgumentException expected) {
			return;
		}
		throw new AssertionError("Expected IllegalArgumentException");
	}
}
