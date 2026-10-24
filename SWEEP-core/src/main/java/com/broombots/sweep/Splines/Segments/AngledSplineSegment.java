/// Copyright (c) 2026 Tobin Rumsey, all rights reserved
/// Comment and debug assist by Claude, Anthropic

package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

/**
 * Follows a Catmull-Rom position curve while interpolating heading independently through waypoint angles.
 * Headings are unwrapped to take the shorter turn, and NaN waypoint angles are treated as zero.
 */
public class AngledSplineSegment implements Segment {
    /** Cubic x-position curve, in inches over normalized segment parameter. */
    private final CatmullRomCubic xCubic;
    /** Cubic y-position curve, in inches over normalized segment parameter. */
    private final CatmullRomCubic yCubic;
    /** Cubic heading curve, in degrees over normalized segment parameter. */
    private final CatmullRomCubic angleCubic;
    /** Increment used to numerically approximate curve distance, as a fraction of the segment. */
    private final double sampleRate = 0.001;
    /** Speed fraction for this segment, taken from its endpoint waypoint. */
    private final double speed;

    /**
     * Creates an angled spline segment through four control waypoints, from the second waypoint to the third.
     * Position uses waypoint coordinates in inches; heading uses waypoint angles in degrees.
     *
     * @param p1 control waypoint before the segment start
     * @param p2 start control waypoint
     * @param p3 end control waypoint; its speed sets the segment speed
     * @param p4 control waypoint after the segment end
     */
    public AngledSplineSegment(Waypoint p1, Waypoint p2, Waypoint p3, Waypoint p4) {
        xCubic = new CatmullRomCubic(p1.getX(), p2.getX(), p3.getX(), p4.getX());
        yCubic = new CatmullRomCubic(p1.getY(), p2.getY(), p3.getY(), p4.getY());
        double a1 = finiteAngle(p1);
        double a2 = unwrapNear(finiteAngle(p2), a1);
        double a3 = unwrapNear(finiteAngle(p3), a2);
        double a4 = unwrapNear(finiteAngle(p4), a3);
        angleCubic = new CatmullRomCubic(a1, a2, a3, a4);
        // comeToStop in MotionProfileProcessor.compileVelocityProfile handles deceleration to zero
        // at BREAK/WAIT boundaries — forcing speed=0 here zeroed the entire ideal map, not just the endpoint.
        speed = p3.getSpeed();
    }
    /** Treats NaN headings on wait and break waypoints as zero for angle interpolation. */
    private static double finiteAngle(Waypoint waypoint){
        double angle = waypoint.getAngle();
        return Double.isNaN(angle) ? 0 : angle;
    }
    /** Shifts an angle by full rotations so interpolation turns within 180 degrees of its reference. */
    private static double unwrapNear(double angle, double reference){
        double diff = (angle - reference) % 360;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return reference + diff;
    }
    /**
     * Evaluates position and interpolated heading at a normalized curve parameter.
     *
     * @param time normalized curve parameter in the range {@code [0, 1]}
     * @return pose at that parameter, with position in inches and heading in degrees
     */
    @Override
    public Pos2D getPosition(double time) {
        time = putInRange(time);
        return new Pos2D(xCubic.evaluate(time), yCubic.evaluate(time), angleCubic.evaluate(time));
    }
    /**
     * Numerically estimates curve length between normalized segment parameters.
     *
     * @param tStart start parameter in the range {@code [0, 1]}
     * @param tEnd end parameter in the range {@code [0, 1]}
     * @return estimated distance in inches
     */
    @Override
    public double calculateDistance(double tStart, double tEnd) {
        tStart = putInRange(tStart);
        tEnd = putInRange(tEnd);
        double distance = 0;
        for (double t = tStart; t < tEnd; t += sampleRate) {
            distance += Pos2D.getDistanceBetweenCoordinates(getPosition(t),getPosition(t+sampleRate));
        }
        return distance;
    }
    /**
     * Gets this segment speed fraction relative to the robot maximum speed.
     *
     * @return speed fraction in the range {@code [0, 1]}
     */
    @Override
    public double getSpeedRate(){
        return speed;
    }
    /** Clamps a segment parameter to the supported curve domain. */
    private double putInRange(double timeUnit) {
        timeUnit = Math.min(timeUnit, 1);
        timeUnit = Math.max(timeUnit, 0);
        return timeUnit;
    }
    /**
     * Gets the cubic x and y coefficient rows for this curve.
     *
     * @return a 2x4 matrix, with x coefficients in row 0 and y coefficients in row 1
     */
    @Override
    public SimpleMatrix getSplineFormula(){
        return xCubic.getCoeffs().combine(1,0, yCubic.getCoeffs());
    }
}
