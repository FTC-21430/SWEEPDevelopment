package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CatmullRomCubicTest {
	@Test
	public void evaluateMatchesEndpoints() {
		CatmullRomCubic cubic = new CatmullRomCubic(0.0, 1.0, 2.0, 3.0);

		assertEquals(1.0, cubic.evaluate(0.0), 1e-9);
		assertEquals(2.0, cubic.evaluate(1.0), 1e-9);
	}

	@Test
	public void derivativeMatchesExpectedStartSlope() {
		CatmullRomCubic cubic = new CatmullRomCubic(0.0, 1.0, 2.0, 3.0);

		assertEquals(1.0, cubic.derivative(0.0), 1e-9);
	}
}
