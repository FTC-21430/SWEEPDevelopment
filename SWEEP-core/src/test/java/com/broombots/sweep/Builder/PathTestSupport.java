package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;

import org.ejml.simple.SimpleMatrix;
import org.junit.Assert;

import java.util.ArrayList;

/**
 * Shared helpers for tests that inspect generated {@link Sequence} paths.
 * Paths are inspected by sampling {@link Sequence#getMovement(double)} so the tests only use public API.
 */
final class PathTestSupport {
	/** Dimensionless builder parameter step used to keep path chord error far below 1e-3 inches. */
	static final double BUILD_SAMPLE_RATE = 0.001;
	/** Verifies that path test support. */
	private PathTestSupport() {
	}

	/**
	 * Builds a sequence from a flat list of start, intermediate, and end poses.
	 * @param params movement limits used to plan the path
	 * @param startThenSplinesThenEnd pose values in x/y inches and heading degrees, with intermediate spline poses
	 * @return the planned sequence
	 */
	static Sequence build(RobotMovementParameters params, double... startThenSplinesThenEnd) {
		SequenceBuilder builder = new SequenceBuilder(params).setSampleRate(BUILD_SAMPLE_RATE);
		int last = startThenSplinesThenEnd.length - 3;
		builder.start(startThenSplinesThenEnd[0], startThenSplinesThenEnd[1], startThenSplinesThenEnd[2]);
		for (int i = 3; i < last; i += 2) {
			builder.splineTo(startThenSplinesThenEnd[i], startThenSplinesThenEnd[i + 1], 1.0);
		}
		builder.end(startThenSplinesThenEnd[last], startThenSplinesThenEnd[last + 1], startThenSplinesThenEnd[last + 2]);
		return builder.build();
	}

	/**
	 * Samples the path at a fixed time step, always including the final time.
	 * @param sequence path to sample
	 * @param step time between samples, in seconds
	 * @return sampled path points, including the endpoint
	 */
	static ArrayList<PathPoint> sample(Sequence sequence, double step) {
		ArrayList<PathPoint> samples = new ArrayList<>();
		double last = sequence.getLastTime();
		for (double t = 0; t < last; t += step) {
			samples.add(sequence.getMovement(t));
		}
		samples.add(sequence.getMovement(last));
		return samples;
	}

	/**
	 * Gets the magnitude of a point's translational velocity.
	 * @param point path point whose velocity is measured
	 * @return speed in inches per second
	 */
	static double speed(PathPoint point) {
		return point.velocity.getMagnitude();
	}

	/**
	 * Gets the direction of travel in degrees, the unit the movement parameters expect.
	 * @param point path point whose velocity direction is measured
	 * @return travel direction in degrees
	 */
	static double travelDirectionDegrees(PathPoint point) {
		return Math.toDegrees(Math.atan2(point.velocity.y, point.velocity.x));
	}

	/**
	 * Measures the distance between two positions.
	 * @param a first position, in inches
	 * @param b second position, in inches
	 * @return distance in inches
	 */
	static double distance(Pos2D a, Pos2D b) {
		return Pos2D.getDistanceBetweenCoordinates(a, b);
	}

	/**
	 * Builds the Catmull-Rom curve SWEEP should follow through the given x,y waypoints.
	 * Each interval uses the previous and next waypoints as tangent controls, clamped at the ends.
	 * With zero starting velocity, the control before the start is the first waypoint after it.
	 * @param waypoints x/y waypoint coordinates in inches
	 * @param samplesPerSegment number of reference points per spline segment
	 * @return sampled ideal curve positions in inches
	 */
	static ArrayList<Pos2D> referenceCurve(double[][] waypoints, int samplesPerSegment) {
		ArrayList<Pos2D> curve = new ArrayList<>();
		int n = waypoints.length - 1;
		for (int k = 1; k <= n; k++) {
			double[] p0 = k == 1 ? waypoints[1] : waypoints[k - 2];
			double[] p1 = waypoints[k - 1];
			double[] p2 = waypoints[k];
			double[] p3 = waypoints[Math.min(k + 1, n)];
			CatmullRomCubic x = new CatmullRomCubic(p0[0], p1[0], p2[0], p3[0]);
			CatmullRomCubic y = new CatmullRomCubic(p0[1], p1[1], p2[1], p3[1]);
			for (int i = 0; i <= samplesPerSegment; i++) {
				double t = (double) i / samplesPerSegment;
				curve.add(new Pos2D(x.evaluate(t), y.evaluate(t), 0));
			}
		}
		return curve;
	}

	/**
	 * Finds the shortest distance from a point to a polyline.
	 * @param point position to measure, in inches
	 * @param polyline sampled path positions, in inches
	 * @return shortest distance in inches
	 */
	static double distanceToPolyline(Pos2D point, ArrayList<Pos2D> polyline) {
		double best = Double.MAX_VALUE;
		for (int i = 1; i < polyline.size(); i++) {
			best = Math.min(best, distanceToSegment(point, polyline.get(i - 1), polyline.get(i)));
		}
		return best;
	}

	/**
	 * Finds the shortest distance from a point to a line segment.
	 * @param p position to measure, in inches
	 * @param a first segment endpoint, in inches
	 * @param b second segment endpoint, in inches
	 * @return shortest distance in inches
	 */
	private static double distanceToSegment(Pos2D p, Pos2D a, Pos2D b) {
		double dx = b.x - a.x;
		double dy = b.y - a.y;
		double lengthSquared = dx * dx + dy * dy;
		double ratio = lengthSquared == 0 ? 0 : ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared;
		ratio = Math.max(0, Math.min(1, ratio));
		return Math.hypot(p.x - (a.x + ratio * dx), p.y - (a.y + ratio * dy));
	}

	/**
	 * Fails if any sampled position or velocity is NaN or infinite.
	 * @param sequence path to inspect; positions are in inches and velocities in inches per second
	 */
	static void assertFinitePoseAndVelocity(Sequence sequence) {
		for (PathPoint point : sample(sequence, 0.01)) {
			boolean finite = Double.isFinite(point.position.x) && Double.isFinite(point.position.y)
					&& Double.isFinite(point.velocity.x) && Double.isFinite(point.velocity.y);
			Assert.assertTrue("Non finite pose or velocity at t=" + point.time, finite);
		}
	}

	/**
	 * Fails if any sampled heading is NaN or infinite.
	 * @param sequence path to inspect; headings are in degrees
	 */
	static void assertFiniteHeading(Sequence sequence) {
		for (PathPoint point : sample(sequence, 0.01)) {
			Assert.assertTrue("Non finite heading at t=" + point.time + " x=" + point.position.x,
					Double.isFinite(point.position.angle));
		}
	}

	/**
	 * Asserts that sampled path times never decrease.
	 * @param sequence path to inspect; sample times are in seconds
	 */
	static void assertTimesNeverDecrease(Sequence sequence) {
		double previous = Double.NEGATIVE_INFINITY;
		for (PathPoint point : sample(sequence, 0.001)) {
			Assert.assertTrue("Time went backwards at " + point.time, point.time >= previous);
			previous = point.time;
		}
	}

	/**
	 * Asserts that sampled translational speeds stay within the configured directional limit.
	 * @param sequence path to inspect; speeds are in inches per second
	 * @param params movement limits queried by travel direction in degrees
	 */
	static void assertSpeedWithinLimit(Sequence sequence, RobotMovementParameters params) {
		for (PathPoint point : sample(sequence, 0.01)) {
			double limit = params.getMaxVelocity(travelDirectionDegrees(point));
			Assert.assertTrue("Commanded " + speed(point) + " in/s over the " + limit + " limit at t=" + point.time,
					speed(point) <= limit * 1.02);
		}
	}

	/**
	 * Measures the time the robot spends stationary within 1e-3 inches of the target.
	 * @param sequence path to inspect
	 * @param target target position, in inches
	 * @return stationary duration in seconds
	 */
	static double restDurationAt(Sequence sequence, Pos2D target) {
		double step = 1e-4;
		int resting = 0;
		for (PathPoint point : sample(sequence, step)) {
			if (distance(point.position, target) <= 1e-3 && speed(point) <= 1e-6) resting++;
		}
		return resting * step;
	}

	/**
	 * Creates robot limits that do not depend on direction so test expectations are exact.
	 * @param maxVelocity maximum translational speed in inches per second
	 * @param maxAcceleration maximum translational acceleration in inches per second squared
	 * @return movement parameters with constant translational limits
	 */
	static RobotMovementParameters constantLimits(double maxVelocity, double maxAcceleration) {
		return new RobotMovementParameters() {
			/**
			 * Verifies that get max velocity.
			 * @param direction requested movement direction in degrees
			 * @return localization packet at the requested position
			 */
			@Override
			public double getMaxVelocity(double direction) {
				return maxVelocity;
			}
			/**
			 * Verifies that get max angle velocity.
			 * @return localization packet at the requested position
			 */
			@Override
			public double getMaxAngleVelocity() {
				return 180;
			}
			/**
			 * Verifies that get max stable acceleration.
			 * @param direction requested movement direction in degrees
			 * @return localization packet at the requested position
			 */
			@Override
			public double getMaxStableAcceleration(double direction) {
				return maxAcceleration;
			}
			/**
			 * Verifies that get max stable angular acceleration.
			 * @param movementMagnitude requested movement magnitude
			 * @param clockwise whether the rotation is clockwise
			 * @return localization packet at the requested position
			 */
			@Override
			public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise) {
				return 200;
			}
			/**
			 * Verifies that get angle full power to error threshold.
			 * @return localization packet at the requested position
			 */
			@Override
			public double getAngleFullPowerToErrorThreshold() {
				return 40;
			}
			/** Verifies that get pidcoefficients. */
			@Override
			public SimpleMatrix getPIDCoefficients() {
				return new SimpleMatrix(3, 3);
			}
		};
	}
}
