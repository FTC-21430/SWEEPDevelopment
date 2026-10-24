package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Regression tests for three Drivetrain bugs: field-centric mode being switched off by the four-argument overload,
 * the stored input being rotated again on every update, and reverse powers not being normalized.
 */
public class DrivetrainTest {
	private static final String[] MOTOR_NAMES = {"fl", "fr", "bl", "br"};
	private static final double EPSILON = 1e-9;

	/** Last raw power each fake motor was given, before any SDK clipping. */
	private final Map<String, Double> powers = new HashMap<>();
	private Drivetrain drivetrain;

	@Before
	public void setUp() {
		final Map<String, DcMotor> motors = new HashMap<>();
		// HardwareMap.get() reaches Android-only static initialisation, so serve the fake motors directly
		HardwareMap hardwareMap = new HardwareMap(null, null) {
			@Override
			@SuppressWarnings("unchecked")
			public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
				return (T) motors.get(deviceName);
			}
		};
		for (final String name : MOTOR_NAMES) {
			InvocationHandler recorder = (proxy, method, args) -> {
				switch (method.getName()) {
					case "setPower":
						powers.put(name, (Double) args[0]);
						return null;
					case "hashCode":
						return name.hashCode();
					case "equals":
						return proxy == args[0];
					case "toString":
						return name;
					default:
						return null;
				}
			};
			DcMotor motor = (DcMotor) Proxy.newProxyInstance(
					DcMotor.class.getClassLoader(), new Class<?>[]{DcMotor.class}, recorder);
			motors.put(name, motor);
		}
		drivetrain = new Drivetrain(hardwareMap, null, false);
	}

	private double[] currentPowers() {
		double[] result = new double[MOTOR_NAMES.length];
		for (int i = 0; i < MOTOR_NAMES.length; i++) result[i] = powers.get(MOTOR_NAMES[i]);
		return result;
	}

	/** The four-argument overload exists to turn field-centric driving on, but it calls the three-argument one that turns it off. */
	@Test
	public void fourArgumentUpdateActuallyDrivesFieldCentric() {
		drivetrain.updateMovementVector(0, 1, 0, 0);
		drivetrain.update();
		double[] headingZero = currentPowers();

		drivetrain.updateMovementVector(0, 1, 0, 90);
		drivetrain.update();
		double[] headingNinety = currentPowers();

		assertFalse("robot yaw had no effect, field-centric mode is off", Arrays.equals(headingZero, headingNinety));
	}

	/** update() overwrites the stored vector with the rotated one, so every loop rotates it again. */
	@Test
	public void repeatedUpdatesWithTheSameInputGiveTheSamePowers() {
		drivetrain.updateMovementVector(0, 1, 0, 90);
		drivetrain.enableFieldCentricDriving(true); // works around the overload bug above to isolate this one
		drivetrain.update();
		double[] first = currentPowers();

		drivetrain.update();
		double[] second = currentPowers();

		assertArrayEquals(first, second, EPSILON);
	}

	/** Normalizing by the largest positive power only, so a reverse command above 1.0 is never scaled down. */
	@Test
	public void reversePowersAreNormalizedLikeForwardPowers() {
		drivetrain.updateMovementVector(-1, 0, -0.5);
		drivetrain.update();
		double[] reverse = currentPowers();

		drivetrain.updateMovementVector(1, 0, 0.5);
		drivetrain.update();
		double[] forward = currentPowers();

		for (double power : reverse) assertTrue("power " + power + " exceeds 1.0", Math.abs(power) <= 1.0 + EPSILON);
		// mirrored input must give mirrored output magnitudes, otherwise the wheel ratios (and the path) change
		assertEquals(Math.abs(forward[0]), Math.abs(reverse[0]), EPSILON);
		assertEquals(Math.abs(forward[2]), Math.abs(reverse[2]), EPSILON);
	}
}
