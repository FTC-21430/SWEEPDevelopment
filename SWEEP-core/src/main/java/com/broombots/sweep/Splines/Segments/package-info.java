/**
 * Contains the time-parameterized curve pieces generated from user-facing waypoints in
 * {@link com.broombots.sweep.Splines}. Movement-segment parameters {@code t} range from 0 to 1; wait segments
 * instead hold a pose for a duration in seconds. {@link com.broombots.sweep.Builder.SequenceBuilder} and the
 * path-generation pipeline convert waypoint anchors into these segments.
 */
package com.broombots.sweep.Splines.Segments;
