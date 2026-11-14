package com.broombots.sweep.integration;

import com.broombots.sweep.Classes.Robot;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/** Verifies alliance access for robot implementations outside the Classes package. */
public class RobotAllianceAccessTest {
	/** Checks that a subclass can use the public alliance type with the protected accessors. */
	@Test
	public void subclassCanChangeAlliance() {
		TestRobot robot = new TestRobot();
		robot.selectAlliance(Robot.Alliance.BLUE);
		assertEquals(Robot.Alliance.BLUE, robot.currentAlliance());
	}

	private static class TestRobot extends Robot {
		void selectAlliance(Alliance alliance) {
			setAlliance(alliance);
		}

		Alliance currentAlliance() {
			return getAlliance();
		}
	}
}
