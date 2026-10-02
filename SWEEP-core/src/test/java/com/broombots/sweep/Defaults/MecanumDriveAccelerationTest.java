package com.broombots.sweep.Defaults;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class MecanumDriveAccelerationTest {
	@Test
	public void movementMagnitudeMatchesCardinalDirection() {
		assertEquals(2.0 * Math.sqrt(2.0), MecanumDriveAcceleration.getMovementMagnitude(0.0, 0.0), 1e-9);
		assertEquals(2.0, MecanumDriveAcceleration.getMovementMagnitude(45.0, 0.0), 1e-9);
	}

	@Test
	public void positiveRotationErrorReducesMagnitude() {
		double noTurn = MecanumDriveAcceleration.getMovementMagnitude(0.0, 0.0);
		double maxTurn = MecanumDriveAcceleration.getMovementMagnitude(0.0, 40.0);

		assertEquals(noTurn * 0.5, maxTurn, 1e-9);
	}
}
