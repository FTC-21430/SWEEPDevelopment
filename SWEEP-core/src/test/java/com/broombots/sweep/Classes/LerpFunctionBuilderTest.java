package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class LerpFunctionBuilderTest {
	@Test
	public void buildCreatesFunctionFromAddedPoints() {
		LerpFunction function = new LerpFunctionBuilder()
				.addPoint(0.0, 0.0)
				.addPoint(1.0, 2.0)
				.build();

		assertNotNull(function);
		assertEquals(1.0, function.getValue(0.5), 1e-9);
	}
}
