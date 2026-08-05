package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.CatmullRomCubic;
import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;
import com.google.gson.internal.bind.JsonAdapterAnnotationTypeAdapterFactory;

import java.util.ArrayList;
import java.util.Comparator;

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

    public MovementMap processPath(Segment[] segments, double sampleRate, MovementPoint startingPoint) {
        MovementMap movementMap = new MovementMap(sampleRate);
        MovementPoint lastPoint = startingPoint;
        for (int i = 0; i < segments.length; i++){
            MovementMap.combine(movementMap, compileVelocityProfile(segments[i], lastPoint));
            lastPoint = movementMap.getAllPoints().get(movementMap.getAllPoints().size()-1);
        }
        return movementMap;
    }
    private MovementMap compileVelocityProfile(Segment segment, MovementPoint startingPoint, double sampleRate) {
        distanceMap = new DistanceMap(segment);
        double startingDistance = 0;
        double endingDistance = distanceMap.getMaxDistance();
        MovementMap idealMap = calculateIdealMovementMap(distanceMap, segment.getSpeedRate(),sampleRate);
        double[] maxCurvature = distanceMap.getSegmentMaxCurvaturePoints();
        double[] simulationPoints = new double[]{
                startingDistance,
                maxCurvature[0],
                maxCurvature[1],
                endingDistance
        };
        ArrayList<MovementMap> simulations = new ArrayList<>();
        for (double point : simulationPoints){
            MovementMap simulation = new MovementMap(sampleRate);
            double currentDistance = point;

            MovementPoint lastPoint;
            if (point == simulationPoints[0]){
                lastPoint = startingPoint;
            }else{
                lastPoint = idealMap.getPoint(point);
            }
            //forward pass
            while (currentDistance <= endingDistance){
                MovementPoint basicResult = simulateStep(sampleRate,currentDistance,)
                currentDistance += sampleRate;
            }
            // backward pass
        }

        //Algorithm steps
        // 1. convert to be in terms of distance - DONE
        // 2. get the max points of curvature.
        // 3. Use the curve of those points to determine the max robot velocity at that curve ( and therefore the slowest theoretically in the spline the robot could move at)
        // 4. Simulate at a sample rate of distance, the fastest the robot would be able to accelerate from each of those distances
        // 5. Compute full velocity profile by comparing each simulation and taking the lowest velocity at each point
        // 6. iterate through the velocity profile to get a full movementmap that represents the segment
        // 7. return that MovementMap which is the velocity profile.
    }


    private MovementMap calculateIdealMovementMap(DistanceMap distanceMap, double segmentSpeedRatio, double sampleRate){
        MovementMap resultingIdealMap = new MovementMap(sampleRate);
        double startDistance = distanceMap.getDistanceBoundsForSegmentIndex(0)[0];
        double endDistance = distanceMap.getDistanceBoundsForSegmentIndex(0)[1];
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

    private MovementPoint simulateStep(double step, double distance, DistanceMap distanceMap, MovementPoint point, MovementMap idealMap){
        // Calculate the new distance along the path (positive step = forward pass, negative step = backward pass)
        double newDistance = distance + step;
        double absStep = Math.abs(step);
        
        // ==== LINEAR VELOCITY CALCULATIONS ====
        // Get heading to the next point along the path
        double heading = getHeadingToCoordinate(point.getPosition(), distanceMap.getPositionAtDistance(newDistance));
        double initialVelocityDirection = Math.toDegrees(Math.atan2(point.getVelY(), point.getVelX()));
        
        // Calculate maximum stable acceleration given current heading and angle error
        double tempAcceleration = movementParameters.getMaxStableAcceleration(heading, point.getPosition().getAngle() + heading - initialVelocityDirection);
        
        // Apply acceleration over the time step to get final velocities
        double finalVelocityX = point.getVelX() + (tempAcceleration * Math.cos(Math.toRadians(heading - point.getPosition().getAngle())) * step);
        double finalVelocityY = point.getVelY() + (tempAcceleration * Math.sin(Math.toRadians(heading - point.getPosition().getAngle())) * step);
        double finalVelocityMagnitude = Math.hypot(finalVelocityX, finalVelocityY);
        
        // Clamp velocities to ideal profile to prevent exceeding path constraints
        if (idealMap.getPoint(newDistance).getVelocityMagnitude() < finalVelocityMagnitude){
            finalVelocityX = idealMap.getPoint(newDistance).getVelX();
            finalVelocityY = idealMap.getPoint(newDistance).getVelY();
        }
        
        // Calculate velocity deltas and position displacement using average velocity
        double deltaVelocityX = finalVelocityX - point.getVelX();
        double deltaVelocityY = finalVelocityY - point.getVelY();
        double finalX = step * (point.getVelX() + (1.0/2.0) * deltaVelocityX);
        double finalY = step * (point.getVelY() + (1.0/2.0) * deltaVelocityY);
        double finalAccelerationX = deltaVelocityX / absStep;
        double finalAccelerationY = deltaVelocityY / absStep;
        
        // ==== ROTATIONAL VELOCITY CALCULATIONS ====
        // Get current angular velocity and calculate angular acceleration
        double currentAngularVelocity = point.getVelAngle();
        double angularAcceleration = movementParameters.getMaxStableAngularAcceleration(currentAngularVelocity);
        
        // Apply angular acceleration over the time step
        double finalVelocityAngle = currentAngularVelocity + (angularAcceleration * step);
        
        // Clamp angular velocity to ideal profile using magnitude comparison for bidirectional motion
        if (Math.abs(idealMap.getPoint(newDistance).getVelAngle()) < Math.abs(finalVelocityAngle)){
            finalVelocityAngle = idealMap.getPoint(newDistance).getVelAngle();
        }
        
        // Calculate angular displacement using average angular velocity
        double deltaVelocityAngle = finalVelocityAngle - currentAngularVelocity;
        double finalAngle = step * (currentAngularVelocity + (1.0/2.0) * deltaVelocityAngle);
        double finalAccelerationAngle = deltaVelocityAngle / absStep;
        
        // ==== CREATE NEW MOVEMENT POINT ====
        MovementPoint newPoint = new MovementPoint(
                new Coordinate(finalX, finalY),
                finalVelocityX,
                finalVelocityY,
                finalVelocityAngle,
                0,
                0,
                0
        );
        
        // Store calculated accelerations in the previous point for trajectory analysis
        point.setAcceleration(finalAccelerationX, finalAccelerationY, finalAccelerationAngle);
        return newPoint;
    }
    private double getHeadingToCoordinate(Coordinate start, Coordinate end){
        double xDifference = end.getX() - start.getX();
        double yDifference = end.getY() - start.getY();
        double baseAngle = Math.atan2(xDifference,yDifference);
        baseAngle *= (180.0/Math.PI); // convert to degrees from RADS
        return baseAngle - start.getAngle(); // TODO: double check the signs.
    }

}

