package com.broombots.sweep.Defaults;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/** Tests baseline angular motion limits and default control thresholds. */
public class DefaultRobotMovementParametersTest {
	/** Verifies that constants match expected defaults. */
	@Test
	public void constantsMatchExpectedDefaults() {
		DefaultRobotMovementParameters parameters = new DefaultRobotMovementParameters();

		assertEquals(30.0, parameters.getMaxAngleVelocity(), 1e-9);
		assertEquals(40.0, parameters.getAngleFullPowerToErrorThreshold(), 1e-9);
		assertEquals(10.0, parameters.getMaxStableAngularAcceleration(1.0, true), 1e-9);
	}
}
