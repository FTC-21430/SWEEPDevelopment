package com.broombots.sweep.Defaults;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/** Tests directional magnitude values for representative mecanum drive directions. */
public class MecanumDriveAccelerationTest {
	/** Verifies that movement magnitude matches cardinal direction. */
	@Test
	public void movementMagnitudeMatchesCardinalDirection() {
		assertEquals(2.0 * Math.sqrt(2.0), MecanumDriveAcceleration.getMovementMagnitude(0.0), 1e-9);
		assertEquals(2.0, MecanumDriveAcceleration.getMovementMagnitude(45.0), 1e-9);
	}
}
