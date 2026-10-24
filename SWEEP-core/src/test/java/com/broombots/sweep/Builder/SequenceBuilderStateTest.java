package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Regression tests for builder state bugs: a missing starting point for {@code start(Pos2D)}, and start times or
 * caller-owned points being modified by {@code start} and {@code build(time)}.
 */
public class SequenceBuilderStateTest {
	private static SequenceBuilder builder() {
		return new SequenceBuilder(new DefaultRobotMovementParameters()).setSampleRate(0.005);
	}

	/** start(Pos2D) used to never store a starting PathPoint, so build() crashed with a NullPointerException. */
	@Test
	public void startWithPos2DCanBeBuilt() {
		Sequence sequence = builder().start(new Pos2D(0, 0, 0)).splineTo(20, 0, 1).end(new Pos2D(40, 0, 0)).build();

		assertTrue(sequence.getLastTime() > 0);
	}

	/** build(time) used to add the offset onto the stored start point, so it leaked into later builds. */
	@Test
	public void buildWithAStartTimeDoesNotAffectLaterBuilds() {
		SequenceBuilder builder = builder().start(0, 0, 0).splineTo(20, 0, 1).end(40, 0, 0);

		double base = builder.build().getLastTime();
		builder.build(5);
		double afterOffsetBuild = builder.build().getLastTime();

		assertEquals(base, afterOffsetBuild, 1e-9);
	}

	/** Building twice with the same offset must give the same result, not 5 s then 10 s. */
	@Test
	public void buildingTwiceWithTheSameStartTimeGivesTheSamePath() {
		SequenceBuilder builder = builder().start(0, 0, 0).splineTo(20, 0, 1).end(40, 0, 0);

		double first = builder.build(5).getLastTime();
		double second = builder.build(5).getLastTime();

		assertEquals(first, second, 1e-9);
	}

	/** start(PathPoint) and build(time) used to write into the caller's PathPoint instead of copying it. */
	@Test
	public void startAndBuildDoNotModifyTheCallersPathPoint() {
		PathPoint point = new PathPoint();
		point.time = 3;
		point.position = new Pos2D(0, 0, 0);
		point.velocity = new Pos2D(0, 0, 0);

		builder().start(point).splineTo(20, 0, 1).end(40, 0, 0).build(5);

		assertEquals("caller's PathPoint time was changed", 3, point.time, 1e-9);
	}
}
