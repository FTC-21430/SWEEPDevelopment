package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.BSplineCubic;
import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.SplineWaypoint;

import org.ejml.simple.SimpleMatrix;

public class AngledSplineSegment implements Segment {
    CatmullRomCubic xCubic;
    CatmullRomCubic yCubic;
    CatmullRomCubic angleCubic;
    private final double sampleRate = 0.001; // units
    private final double speed;

    /**
     * Creates a following spline segment
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
    // Wait and break waypoints carry a NaN angle (resolved later in PathProcessor); treat it as 0 like SplineWaypoint does
    private static double finiteAngle(Waypoint waypoint){
        double angle = waypoint.getAngle();
        return Double.isNaN(angle) ? 0 : angle;
    }
    // Shifts angle by multiples of 360 so it is within 180 degrees of reference, making the cubic turn the short way
    private static double unwrapNear(double angle, double reference){
        double diff = (angle - reference) % 360;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return reference + diff;
    }
    /**
     * @param time absolute time ( must range between 0-1 )
     * @return pose at the requested time
     */
    @Override
    public Pos2D getPosition(double time) {
        time = putInRange(time);
        return new Pos2D(xCubic.evaluate(time), yCubic.evaluate(time), angleCubic.evaluate(time));
    }
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
    @Override
    public double getSpeedRate(){
        return speed;
    }
    private double putInRange(double timeUnit) {
        timeUnit = Math.min(timeUnit, 1);
        timeUnit = Math.max(timeUnit, 0);
        return timeUnit;
    }
    @Override
    public SimpleMatrix getSplineFormula(){
        return xCubic.getCoeffs().combine(1,0, yCubic.getCoeffs());
    }
}
