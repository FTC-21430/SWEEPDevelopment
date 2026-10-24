/**
 * Interprets compiled sequences and converts their targets into drivetrain powers.
 * <p>
 * The data flow begins with {@link com.broombots.sweep.Builder.SequenceBuilder}, which uses
 * {@link com.broombots.sweep.Builder.PathProcessor} to compile a {@link com.broombots.sweep.Builder.Sequence}.
 * {@link com.broombots.sweep.Movement.SequenceInterpreter} executes that sequence and supplies target state to
 * {@link com.broombots.sweep.Movement.RobotMovementController}, which produces robot-centric drivetrain powers.
 */
package com.broombots.sweep.Movement;
