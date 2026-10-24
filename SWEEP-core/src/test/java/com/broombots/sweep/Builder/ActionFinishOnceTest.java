package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Robot;
import com.broombots.sweep.Classes.SWEEPAction;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;

/** Regression test: a Sequence must end its actions through the guarded finish() so end() never runs twice. */
public class ActionFinishOnceTest {
	private static class TestRobot extends Robot {
	}

	/** Completes immediately (hold time 0) and counts how many times end() runs. */
	private static class CountingAction extends SWEEPAction {
		int endCount = 0;

		/** Creates an immediately-completing action that counts end callbacks. */
		CountingAction() {
			super(new TestRobot(), new Pos2D(0, 0, 0), 0, 1, 0);
		}
		/** Executes the simulated action. */
		@Override
		public void execute() {
		}
		/** Runs one simulated action update. */
		@Override
		public void process() {
		}
		/** Ends the simulated action. */
		@Override
		public void end() {
			endCount++;
		}
	}

	/**
	 * Action.interrupt() ends the action through the guarded finish(). The Sequence then saw the action as complete
	 * and called end() itself, so cleanup code ran twice.
	 */
	@Test
	public void endRunsOnlyOnceWhenAnActionIsInterruptedWhileRunningInASequence() {
		CountingAction action = new CountingAction();
		Sequence sequence = new Sequence(new ArrayList<PathPoint>(), new SWEEPAction[]{action}, new ArrayList<Double>());
		LocalizationPacket atTrigger = new LocalizationPacket(0, 0, 0, 0, 0);

		sequence.updateActions(atTrigger, 0);
		action.interrupt();
		sequence.updateActions(atTrigger, 0);

		assertEquals(1, action.endCount);
	}
}
