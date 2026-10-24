package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.SplineWaypoint;
import com.broombots.sweep.Splines.Segments.FollowSplineSegment;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;

/** Regression tests for direction units passed into terminal speed limits. */
public class PathProcessorDirectionUnitsTest {
	/** Verifies terminal speed uses a travel direction in degrees. */
	@Test
	public void terminalSpeedUsesDegreesForDirectionalVelocityLimit() {
		DefaultRobotMovementParameters parameters = new DefaultRobotMovementParameters();
		PathProcessor processor = new PathProcessor(parameters);
		PathPoint startingPoint = new PathPoint();
		startingPoint.position = new Pos2D(0, 0, 0);
		startingPoint.velocity = new Pos2D(0, 0, 0);
		startingPoint.time = 0;
		Segment diagonal = new FollowSplineSegment(
				new SplineWaypoint(0, 0, 1),
				new SplineWaypoint(0, 0, 1),
				new SplineWaypoint(10, 10, 1),
				new SplineWaypoint(20, 20, 1));

		ArrayList<Object> result = processor.processPath(
				new Segment[]{diagonal}, 0.01, startingPoint, 1.0);
		@SuppressWarnings("unchecked")
		ArrayList<PathPoint> path = (ArrayList<PathPoint>) result.get(0);
		PathPoint terminalPoint = path.get(path.size() - 1);

		assertEquals(45.0, Math.toDegrees(Math.atan2(
				terminalPoint.position.y - path.get(path.size() - 2).position.y,
				terminalPoint.position.x - path.get(path.size() - 2).position.x)), 1e-6);
		assertEquals(parameters.getMaxVelocity(45.0),
				terminalPoint.velocity.getMagnitude(), 1e-6);
	}
}
