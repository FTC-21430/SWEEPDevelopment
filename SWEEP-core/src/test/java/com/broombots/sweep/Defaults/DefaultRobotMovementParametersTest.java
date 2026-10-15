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
}
