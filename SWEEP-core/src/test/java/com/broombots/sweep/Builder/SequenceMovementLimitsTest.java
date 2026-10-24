package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;

/**
 * Verifies that generated translational motion respects configured speed and acceleration limits.
 * These limits keep path following within the robot's physical capabilities.
 */
public class SequenceMovementLimitsTest {
	/** Interval between sampled commands, in seconds. */
	private static final double TIME_STEP = 0.01; // seconds between sampled movement commands
	/** Relative allowance for floating point and interpolation error. */
	private static final double LIMIT_SLACK = 1.02; // allows for floating point and interpolation error

	/** Test route coordinates are inches and the final pose heading is degrees. */
	private static final double[] CURVE = {-30, -40, 0, -10, -10, 20, 0, 40, 30, 30, 50, 90};

	/**
	 * Asserts sampled speed remains within the movement limit for each travel direction.
	 * @param sequence path to inspect; speeds are in inches per second
	 * @param params movement limits, queried using directions in degrees
	 */
	private static void assertSpeedWithinLimit(Sequence sequence, RobotMovementParameters params) {
		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			double limit = params.getMaxVelocity(PathTestSupport.travelDirectionDegrees(point));
			assertTrue("Commanded " + PathTestSupport.speed(point) + " in/s over the " + limit + " limit at t="
					+ point.time, PathTestSupport.speed(point) <= limit * LIMIT_SLACK);
		}
	}

	/**
	 * Asserts sampled changes in speed remain within the directional acceleration limit.
	 * @param sequence path to inspect; speed is in inches per second and time in seconds
	 * @param params movement limits, queried using directions in degrees
	 */
	private static void assertAccelerationWithinLimit(Sequence sequence, RobotMovementParameters params) {
		ArrayList<PathPoint> samples = PathTestSupport.sample(sequence, TIME_STEP);
		for (int i = 1; i < samples.size(); i++) {
			PathPoint previous = samples.get(i - 1);
			PathPoint current = samples.get(i);
			double dt = current.time - previous.time;
			if (dt <= 0) continue;
			double acceleration = Math.abs(PathTestSupport.speed(current) - PathTestSupport.speed(previous)) / dt;
			double limit = params.getMaxStableAcceleration(PathTestSupport.travelDirectionDegrees(current));
			assertTrue("Commanded " + acceleration + " in/s^2 over the " + limit + " limit at t=" + current.time,
					acceleration <= limit * LIMIT_SLACK);
		}
	}

	/** The 2% slack accommodates numerical sampling and interpolation while enforcing the configured speed cap. */
	@Test
	public void speedStaysWithinDefaultRobotLimits() {
		RobotMovementParameters params = new DefaultRobotMovementParameters();

		assertSpeedWithinLimit(PathTestSupport.build(params, CURVE), params);
	}
	/** Verifies that acceleration stays within default robot limits. */
	@Test
	public void accelerationStaysWithinDefaultRobotLimits() {
		RobotMovementParameters params = new DefaultRobotMovementParameters();

		assertAccelerationWithinLimit(PathTestSupport.build(params, CURVE), params);
	}
	/** Verifies that speed stays within custom limits. */
	@Test
	public void speedStaysWithinCustomLimits() {
		RobotMovementParameters params = PathTestSupport.constantLimits(20, 10);

		assertSpeedWithinLimit(PathTestSupport.build(params, CURVE), params);
	}
	/** Verifies that acceleration stays within custom limits. */
	@Test
	public void accelerationStaysWithinCustomLimits() {
		RobotMovementParameters params = PathTestSupport.constantLimits(20, 10);

		assertAccelerationWithinLimit(PathTestSupport.build(params, CURVE), params);
	}
	/** Verifies that straight path reaches top speed when long enough. */
	@Test
	public void straightPathReachesTopSpeedWhenLongEnough() {
		RobotMovementParameters params = PathTestSupport.constantLimits(20, 40);
		Sequence sequence = PathTestSupport.build(params, -60, 0, 0, -20, 0, 20, 0, 60, 0, 0);
		double peak = 0;

		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			peak = Math.max(peak, PathTestSupport.speed(point));
		}
		assertTrue("Peak speed was only " + peak, peak >= 20 * 0.98);
	}
	/** Verifies that lower limits make the same path take longer. */
	@Test
	public void lowerLimitsMakeTheSamePathTakeLonger() {
		Sequence fast = PathTestSupport.build(PathTestSupport.constantLimits(40, 40), CURVE);
		Sequence slow = PathTestSupport.build(PathTestSupport.constantLimits(20, 10), CURVE);

		assertTrue(slow.getLastTime() > fast.getLastTime());
	}
	/** Verifies that travel time is at least the distance divided by top speed. */
	@Test
	public void travelTimeIsAtLeastTheDistanceDividedByTopSpeed() {
		double topSpeed = 20;
		Sequence sequence = PathTestSupport.build(PathTestSupport.constantLimits(topSpeed, 40),
				-60, 0, 0, 0, 0, 60, 0, 0);

		// 120 inches must take at least 6 seconds, plus the final hold at the end waypoint
		assertTrue(sequence.getLastTime() >= 120 / topSpeed);
	}
	/** Verifies that lower spline speed ratio lowers commanded speed until the final approach. */
	@Test
	public void lowerSplineSpeedRatioLowersCommandedSpeedUntilTheFinalApproach() {
		RobotMovementParameters params = PathTestSupport.constantLimits(40, 40);
		Sequence sequence = new SequenceBuilder(params).setSampleRate(PathTestSupport.BUILD_SAMPLE_RATE)
				.start(-60, 0, 0).splineTo(-20, 0, 0.5).splineTo(20, 0, 0.5).end(60, 0, 0).build();
		double peak = 0;

		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			if (point.position.x > 20) break; // the final segment to END is not scaled by a spline speed ratio
			peak = Math.max(peak, PathTestSupport.speed(point));
		}
		assertTrue("Peak speed was " + peak, peak <= 40 * 0.5 * LIMIT_SLACK);
	}
}
