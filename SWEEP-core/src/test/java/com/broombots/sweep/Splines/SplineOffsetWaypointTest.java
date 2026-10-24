package com.broombots.sweep.Splines;

import com.broombots.sweep.Classes.Waypoint;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SplineOffsetWaypointTest {
	@Test
	public void constructorPreservesCoordinateOffsetAndSpeed() {
		SplineOffsetWaypoint waypoint = new SplineOffsetWaypoint(2.0, 4.0, 180.0, 0.75);

		assertEquals(2.0, waypoint.getX(), 1e-9);
		assertEquals(4.0, waypoint.getY(), 1e-9);
		assertEquals(180.0, waypoint.getHeadingOffset(), 1e-9);
		assertEquals(0.75, waypoint.getSpeed(), 1e-9);
		assertEquals(Waypoint.WaypointType.SPLINE_OFFSET, waypoint.getType());
	}

	@Test
	public void otherWaypointTypesHaveNoHeadingOffset() {
		assertEquals(0.0, new SplineWaypoint(1, 1, 1).getHeadingOffset(), 1e-9);
		assertEquals(0.0, new SplineAngleWaypoint(1, 1, 90, 1).getHeadingOffset(), 1e-9);
	}
}
