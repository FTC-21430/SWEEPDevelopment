package com.broombots.sweep.Classes;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class LocalizationPacketTest {
	@Test
	public void gettersPreserveConstructorValues() {
		LocalizationPacket packet = new LocalizationPacket(1.5, -2.5, 0.75, 4.0, -5.0);

		assertEquals(1.5, packet.getX(), 1e-9);
		assertEquals(-2.5, packet.getY(), 1e-9);
		assertEquals(0.75, packet.getYaw(), 1e-9);
		assertEquals(4.0, packet.getVelX(), 1e-9);
		assertEquals(-5.0, packet.getVelY(), 1e-9);
	}

	@Test
	public void getCoordinateReturnsMatchingPose() {
		LocalizationPacket packet = new LocalizationPacket(8.0, 9.0, 1.25, 0.0, 0.0);

		Pos2D coordinate = packet.getCoordinate();
		assertEquals(8.0, coordinate.x, 1e-9);
		assertEquals(9.0, coordinate.y, 1e-9);
		assertEquals(1.25, coordinate.angle, 1e-9);
	}
}
