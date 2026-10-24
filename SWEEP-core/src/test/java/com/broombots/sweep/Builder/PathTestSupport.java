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
	/** Parameter step used when building paths, small enough that chord error is far below 1e-3 inches. */
	static final double BUILD_SAMPLE_RATE = 0.001;

	private PathTestSupport() {
	}

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

	/** Samples the path at a fixed time step, always including the final time. */
	static ArrayList<PathPoint> sample(Sequence sequence, double step) {
		ArrayList<PathPoint> samples = new ArrayList<>();
		double last = sequence.getLastTime();
		for (double t = 0; t < last; t += step) {
			samples.add(sequence.getMovement(t));
		}
		samples.add(sequence.getMovement(last));
		return samples;
	}

	static double speed(PathPoint point) {
		return point.velocity.getMagnitude();
	}

	/** Direction of travel in degrees, the unit the movement parameters expect. */
	static double travelDirectionDegrees(PathPoint point) {
		return Math.toDegrees(Math.atan2(point.velocity.y, point.velocity.x));
	}

	static double distance(Pos2D a, Pos2D b) {
		return Pos2D.getDistanceBetweenCoordinates(a, b);
	}

	/**
	 * Builds the Catmull-Rom curve SWEEP should follow through the given x,y waypoints.
	 * Each interval uses the previous and next waypoints as tangent controls, clamped at the ends.
	 * With zero starting velocity, the control before the start is the first waypoint after it.
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

	/** Shortest distance from a point to a polyline. */
	static double distanceToPolyline(Pos2D point, ArrayList<Pos2D> polyline) {
		double best = Double.MAX_VALUE;
		for (int i = 1; i < polyline.size(); i++) {
			best = Math.min(best, distanceToSegment(point, polyline.get(i - 1), polyline.get(i)));
		}
		return best;
	}

	private static double distanceToSegment(Pos2D p, Pos2D a, Pos2D b) {
		double dx = b.x - a.x;
		double dy = b.y - a.y;
		double lengthSquared = dx * dx + dy * dy;
		double ratio = lengthSquared == 0 ? 0 : ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared;
		ratio = Math.max(0, Math.min(1, ratio));
		return Math.hypot(p.x - (a.x + ratio * dx), p.y - (a.y + ratio * dy));
	}

	/** Fails if any sampled position or velocity is NaN or infinite. */
	static void assertFinitePoseAndVelocity(Sequence sequence) {
		for (PathPoint point : sample(sequence, 0.01)) {
			boolean finite = Double.isFinite(point.position.x) && Double.isFinite(point.position.y)
					&& Double.isFinite(point.velocity.x) && Double.isFinite(point.velocity.y);
			Assert.assertTrue("Non finite pose or velocity at t=" + point.time, finite);
		}
	}

	/** Fails if any sampled heading is NaN or infinite. */
	static void assertFiniteHeading(Sequence sequence) {
		for (PathPoint point : sample(sequence, 0.01)) {
			Assert.assertTrue("Non finite heading at t=" + point.time + " x=" + point.position.x,
					Double.isFinite(point.position.angle));
		}
	}

	static void assertTimesNeverDecrease(Sequence sequence) {
		double previous = Double.NEGATIVE_INFINITY;
		for (PathPoint point : sample(sequence, 0.001)) {
			Assert.assertTrue("Time went backwards at " + point.time, point.time >= previous);
			previous = point.time;
		}
	}

	static void assertSpeedWithinLimit(Sequence sequence, RobotMovementParameters params) {
		for (PathPoint point : sample(sequence, 0.01)) {
			double limit = params.getMaxVelocity(travelDirectionDegrees(point));
			Assert.assertTrue("Commanded " + speed(point) + " in/s over the " + limit + " limit at t=" + point.time,
					speed(point) <= limit * 1.02);
		}
	}

	/** Total time the robot spends stationary within 1e-3 inches of the target. */
	static double restDurationAt(Sequence sequence, Pos2D target) {
		double step = 1e-4;
		int resting = 0;
		for (PathPoint point : sample(sequence, step)) {
			if (distance(point.position, target) <= 1e-3 && speed(point) <= 1e-6) resting++;
		}
		return resting * step;
	}

	/** Robot limits that do not depend on direction so test expectations are exact. */
	static RobotMovementParameters constantLimits(double maxVelocity, double maxAcceleration) {
		return new RobotMovementParameters() {
			@Override
			public double getMaxVelocity(double direction) {
				return maxVelocity;
			}

			@Override
			public double getMaxAngleVelocity() {
				return 180;
			}

			@Override
			public double getMaxStableAcceleration(double direction) {
				return maxAcceleration;
			}

			@Override
			public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise) {
				return 200;
			}

			@Override
			public double getAngleFullPowerToErrorThreshold() {
				return 40;
			}

			@Override
			public SimpleMatrix getPIDCoefficients() {
				return new SimpleMatrix(3, 3);
			}
		};
	}
}
