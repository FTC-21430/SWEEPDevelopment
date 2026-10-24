package com.broombots.sweep.Classes;

import java.util.ArrayList;

import org.junit.Test;
import static org.junit.Assert.assertEquals;

/** Tests constant and linear interpolation, endpoint clamping, and invalid list sizes. */
public class LerpFunctionTest {
	/** Verifies that get value constant value test. */
	@Test
	public void getValue_constantValueTest() {
		LerpFunction function = new LerpFunctionBuilder()
				.addPoint(0.0, 1.0)
				.build();

		assertEquals(1.0, function.getValue(0), 1e-9);
		assertEquals(1.0, function.getValue(100), 1e-9);
		assertEquals(1.0, function.getValue(-100), 1e-9);
	}
	/** Verifies that get value linear interp test. */
	@Test
	public void getValue_linearInterpTest() {
		LerpFunction function = new LerpFunctionBuilder()
				.addPoint(0.0, 0.0)
				.addPoint(1.0, 1.0)
				.build();
		assertEquals(0.0, function.getValue(0.0), 1e-9);
		assertEquals(0.5, function.getValue(0.5), 1e-9);
		assertEquals(1.0, function.getValue(1.0), 1e-9);
	}
	/** Verifies that get value handles middle points. */
	@Test
	public void getValue_handlesMiddlePoints() {
		LerpFunction function = new LerpFunctionBuilder()
				.addPoint(0.0, 0.0)
				.addPoint(5.0, 50.0)
				.addPoint(10.0, 100.0)
				.build();

		assertEquals(75.0, function.getValue(7.5), 1e-9);
	}
	/** Verifies that constructor rejects mismatched lists. */
	@Test(expected = IllegalArgumentException.class)
	public void constructorRejectsMismatchedLists() {
		ArrayList<Double> keyedPoints = new ArrayList<>();
		ArrayList<Double> resultValues = new ArrayList<>();
		resultValues.add(1.0);

		new LerpFunction(keyedPoints, resultValues);
	}
}
