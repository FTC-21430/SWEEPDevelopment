/**
 * Builds and compiles robot movement sequences from waypoints and actions.
 * <p>
 * {@link com.broombots.sweep.Builder.SequenceBuilder} defines the route and creates {@link com.broombots.sweep.Builder.Sequence} data
 * through {@link com.broombots.sweep.Builder.PathProcessor}. The compiled sequence is then run by
 * {@link com.broombots.sweep.Movement.SequenceInterpreter}, which supplies targets to
 * {@link com.broombots.sweep.Movement.RobotMovementController} for conversion into drivetrain powers.
 */
package com.broombots.sweep.Builder;
