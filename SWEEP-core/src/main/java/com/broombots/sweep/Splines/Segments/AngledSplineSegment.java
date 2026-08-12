package com.broombots.sweep.Splines.Segments;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Coordinate;
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
        angleCubic = new CatmullRomCubic(p1.getAngle(), p2.getAngle(), p3.getAngle(), p4.getAngle());
        // TODO: forcing speed=0 here caps segment.getSpeedRate() -> calculateIdealMovementMap's
        // segmentSpeedRatio to 0 for the ENTIRE segment (maxVelocity = ... * segmentSpeedRatio * ...
        // at every sampled distance, not just near the end), which plans the whole segment leading
        // into a BREAK/WAIT at zero velocity instead of only decelerating near the stop. Now that
        // MotionProfileProcessor.compileVelocityProfile has a comeToStop boundary condition that
        // backward-simulates a proper decel-to-zero at the segment's true end, this blanket zero
        // is likely redundant with (and conflicts with) that mechanism -- revisit whether this
        // override should be removed once comeToStop is fully wired through processPath.
        if (p4.getType() == Waypoint.WaypointType.BREAK || p4.getType() == Waypoint.WaypointType.WAIT){
            speed = 0;
        }else{
            speed = p3.getSpeed();
        }
    }
    /**
     * @param time absolute time ( must range between 0-1 )
     * @return pose at the requested time
     */
    @Override
    public Coordinate getPosition(double time) {
        time = putInRange(time);
        return new Coordinate(xCubic.evaluate(time), yCubic.evaluate(time), angleCubic.evaluate(time));
    }
    @Override
    public double calculateDistance(double tStart, double tEnd) {
        tStart = putInRange(tStart);
        tEnd = putInRange(tEnd);
        double distance = 0;
        for (double t = tStart; t < tEnd; t += sampleRate) {
            distance += Coordinate.getDistanceBetweenCoordinates(getPosition(t),getPosition(t+sampleRate));
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
