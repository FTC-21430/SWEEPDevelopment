package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;

import java.util.ArrayList;

public class MotionProfileProcessor {
    private RobotMovementParameters movementParameters;
    private DistanceMap distanceMap;
    private double curvatureEffect = 20; //TODO Tune this value in

    public MotionProfileProcessor(RobotMovementParameters movementParameters) {
        // Initialize the motion profile processor with the given movement parameters
        this.movementParameters = movementParameters;
    }

    public void updateMovementParameters(RobotMovementParameters movementParameters) {
        // Update the movement parameters for the motion profile processor
        this.movementParameters = movementParameters;
    }

    public MovementMap processPath(Segment[] segments, double sampleRate) {
        MovementMap movementMap = new MovementMap(sampleRate);
        distanceMap = new DistanceMap(segments);

        MovementMap idealMap = new MovementMap(sampleRate);
        for (int i = 0; i < segments.length; i++){
            MovementMap.combine(idealMap, calculateIdealMovementMap(i, distanceMap, segments[i].getSpeedRate(),sampleRate));
        }

        return movementMap;
    }

    private MovementMap calculateIdealMovementMap(int segmentIdx, DistanceMap distanceMap, double segmentSpeedRatio, double sampleRate){
        MovementMap resultingIdealMap = new MovementMap(sampleRate);
        double startDistance = distanceMap.getDistanceBoundsForSegmentIndex(segmentIdx)[0];
        double endDistance = distanceMap.getDistanceBoundsForSegmentIndex(segmentIdx)[1];
        Coordinate lastPoint = distanceMap.getPositionAtDistance(startDistance);
        for (double p = startDistance + sampleRate; p < endDistance; p += sampleRate) {
            double headingDegrees = getHeadingToCoordinate(lastPoint, distanceMap.getPositionAtDistance(p));
            double maxVelocity = movementParameters.getMaxVelocity(headingDegrees) * segmentSpeedRatio * curvatureEffect / (distanceMap.getCurvatureAtDistance(p)+1);
            resultingIdealMap.addMovementPoint(
                new MovementPoint(
                    lastPoint,
                    Math.cos(Math.toRadians(headingDegrees)) * maxVelocity,
                    Math.sin(Math.toRadians(headingDegrees)) * maxVelocity,
                    movementParameters.getAngleVelocity() * segmentSpeedRatio,
                    0,
                    0,
                    0
                ));
        }
        return resultingIdealMap;
    }
    private ArrayList<Integer> getSlowPoints(MovementMap idealMap, double sampleRate){
        ArrayList<MovementPoint> points = idealMap.getAllPoints();
        ArrayList<Integer> allSlowIndex = new ArrayList<>();
        allSlowIndex.add(0);
        int size = points.size();
        allSlowIndex.add(size -1);
        for (int i = 1; i < size-1; i++){
            MovementPoint last = points.get(i-1);
            MovementPoint point = points.get(i);
            MovementPoint next = points.get(i+1);

            double l = last.getVelocityMagnitude();
            double c = point.getVelocityMagnitude();
            double n = next.getVelocityMagnitude();

            if (l > c && n >= c){
                allSlowIndex.add(i);
            }
            if (n > c && l >= c){
                allSlowIndex.add(i);
            }
        }
        return allSlowIndex;
    }

    private MovementPoint simulateStep(double timeStep, DistanceMap distanceMap, MovementPoint point, MovementMap idealMap){
        movementParameters.getMaxStableAcceleration()
    }
    private double getHeadingToCoordinate(Coordinate start, Coordinate end){
        double xDifference = end.getX() - start.getX();
        double yDifference = end.getY() - start.getY();
        double baseAngle = Math.atan2(xDifference,yDifference);
        baseAngle *= (180.0/Math.PI); // convert to degrees from RADS
        return baseAngle - start.getAngle(); // TODO: double check the signs.
    }
    private void compileVelocityProfile(Segment segment) {

        //Algorithm steps
        // 1. convert to be in terms of distance - DONE

        // 3. Use the curve of those points to determine the max robot velocity at that curve ( and therefore the slowest theoretically in the spline the robot could move at)
        // 4. Simulate at a sample rate of distance, the fastest the robot would be able to accelerate from each of those distances
        // 5. Compute full velocity profile by comparing each simulation and taking the lowest velocity at each point
        // 6. iterate through the velocity profile to get a full movementmap that represents the segment
        // 7. return that MovementMap which is the velocity profile.
    }
}

