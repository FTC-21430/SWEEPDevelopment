package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;

public class SequenceMovementLimitsTest {
	private static final double TIME_STEP = 0.01; // seconds between sampled movement commands
	private static final double LIMIT_SLACK = 1.02; // allows for floating point and interpolation error

	private static final double[] CURVE = {-30, -40, 0, -10, -10, 20, 0, 40, 30, 30, 50, 90};

	private static void assertSpeedWithinLimit(Sequence sequence, RobotMovementParameters params) {
		for (PathPoint point : PathTestSupport.sample(sequence, TIME_STEP)) {
			double limit = params.getMaxVelocity(PathTestSupport.travelDirectionDegrees(point));
			assertTrue("Commanded " + PathTestSupport.speed(point) + " in/s over the " + limit + " limit at t="
					+ point.time, PathTestSupport.speed(point) <= limit * LIMIT_SLACK);
		}
	}

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

	@Test
	public void speedStaysWithinDefaultRobotLimits() {
		RobotMovementParameters params = new DefaultRobotMovementParameters();

		assertSpeedWithinLimit(PathTestSupport.build(params, CURVE), params);
	}

	@Test
	public void accelerationStaysWithinDefaultRobotLimits() {
		RobotMovementParameters params = new DefaultRobotMovementParameters();

		assertAccelerationWithinLimit(PathTestSupport.build(params, CURVE), params);
	}

	@Test
	public void speedStaysWithinCustomLimits() {
		RobotMovementParameters params = PathTestSupport.constantLimits(20, 10);

		assertSpeedWithinLimit(PathTestSupport.build(params, CURVE), params);
	}

	@Test
	public void accelerationStaysWithinCustomLimits() {
		RobotMovementParameters params = PathTestSupport.constantLimits(20, 10);

		assertAccelerationWithinLimit(PathTestSupport.build(params, CURVE), params);
	}

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

	@Test
	public void lowerLimitsMakeTheSamePathTakeLonger() {
		Sequence fast = PathTestSupport.build(PathTestSupport.constantLimits(40, 40), CURVE);
		Sequence slow = PathTestSupport.build(PathTestSupport.constantLimits(20, 10), CURVE);

		assertTrue(slow.getLastTime() > fast.getLastTime());
	}

	@Test
	public void travelTimeIsAtLeastTheDistanceDividedByTopSpeed() {
		double topSpeed = 20;
		Sequence sequence = PathTestSupport.build(PathTestSupport.constantLimits(topSpeed, 40),
				-60, 0, 0, 0, 0, 60, 0, 0);

		// 120 inches must take at least 6 seconds, plus the final hold at the end waypoint
		assertTrue(sequence.getLastTime() >= 120 / topSpeed);
	}

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
