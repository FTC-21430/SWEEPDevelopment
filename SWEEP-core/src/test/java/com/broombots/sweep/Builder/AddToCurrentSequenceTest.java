package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.broombots.sweep.Movement.SequenceInterpreter;

import org.junit.Test;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Tests for appending movement to the sequence a SequenceInterpreter is running. */
public class AddToCurrentSequenceTest {
	private static SequenceBuilder builder() {
		return new SequenceBuilder(new DefaultRobotMovementParameters()).setSampleRate(0.005);
	}

	/** addToCurrentSequence used to be an empty method, so the appended movement was silently dropped. */
	@Test
	public void appendedMovementKeepsTheInterpreterActiveAfterTheFirstSequenceFinishes() throws InterruptedException {
		Sequence first = builder().start(0, 0, 0).end(4, 0, 0).build();
		SequenceBuilder second = builder().start(4, 0, 0).end(60, 0, 0);
		SequenceInterpreter interpreter = new SequenceInterpreter(new DefaultRobotMovementParameters());

		double firstDuration = first.getLastTime(); // read before appending, the append lengthens the same Sequence
		interpreter.startPath(first);
		interpreter.addToCurrentSequence(second);
		Thread.sleep((long) (firstDuration * 1000) + 150);
		interpreter.update(new LocalizationPacket(4, 0, 0, 0, 0));

		assertTrue("appended sequence was ignored", interpreter.hasActiveSequence());
	}

	@Test
	public void addingWithoutAnActiveSequenceFailsLoudly() {
		SequenceInterpreter interpreter = new SequenceInterpreter(new DefaultRobotMovementParameters());

		try {
			interpreter.addToCurrentSequence(builder().start(0, 0, 0).end(10, 0, 0));
			fail("Expected IllegalStateException");
		} catch (IllegalStateException expected) {
			assertTrue(expected.getMessage().contains("startPath"));
		}
	}
}
