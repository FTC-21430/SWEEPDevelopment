package com.broombots.sweep.Defaults;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class DefaultRobotMovementParametersTest {
	@Test
	public void constantsMatchExpectedDefaults() {
		DefaultRobotMovementParameters parameters = new DefaultRobotMovementParameters();

		assertEquals(300.0, parameters.getMaxAngleVelocity(), 1e-9);
		assertEquals(40.0, parameters.getAngleFullPowerToErrorThreshold(), 1e-9);
		assertEquals(200.0, parameters.getMaxStableAngularAcceleration(1.0, true), 1e-9);
	}

	@Test
	public void maxVelocityAndAccelerationScaleWithTurnError() {
		DefaultRobotMovementParameters parameters = new DefaultRobotMovementParameters(10.0);

		double noTurnVelocity = parameters.getMaxVelocity(0.0, 0.0);
		double halfTurnVelocity = parameters.getMaxVelocity(0.0, 40.0);
		double noTurnAcceleration = parameters.getMaxStableAcceleration(0.0, 0.0);
		double halfTurnAcceleration = parameters.getMaxStableAcceleration(0.0, 40.0);

		assertEquals(noTurnVelocity * 0.5, halfTurnVelocity, 1e-9);
		assertEquals(noTurnAcceleration * 0.5, halfTurnAcceleration, 1e-9);
	}
}
