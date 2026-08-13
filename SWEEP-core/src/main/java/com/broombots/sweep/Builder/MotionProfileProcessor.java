package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.Segments.WaitSegment;

import java.util.ArrayList;
import java.util.Collections;

public class MotionProfileProcessor {
    private RobotMovementParameters movementParameters;
    private final double curvatureEffect = 20; //TODO Tune this value in

    public MotionProfileProcessor(RobotMovementParameters movementParameters) {
        // Initialize the motion profile processor with the given movement parameters
        this.movementParameters = movementParameters;
    }

    public void updateMovementParameters(RobotMovementParameters movementParameters) {
        // Update the movement parameters for the motion profile processor
        this.movementParameters = movementParameters;
    }

    /**
     * Compiles the full path into a finalized, time-indexed MovementMap that the robot can
     * actually follow. Segments are first compiled and combined in distance-space (using
     * distanceSampleRate), then rendered through time (using timeSampleRate) so that the
     * returned MovementMap's unit is seconds rather than distance.
     * @param segments the path segments to compile, in order
     * @param sampleRate the resolution that the path will be sampled and rendered. (both distance steps and time steps use the same value)
     * @param startingPoint the MovementPoint the robot starts at
     * @return the finalized MovementMap, indexed by time (seconds)
     */
    public MovementMap processPath(Segment[] segments, double sampleRate, MovementPoint startingPoint) {
        if (sampleRate <= 0.0) throw new IllegalArgumentException("SampleRate must be positive");
        System.out.println("SWEEP MotionProfileProcessor: Starting to process path with " + segments.length + " segments at sample rate " + sampleRate);
        MovementMap movementMap = new MovementMap(sampleRate);
        MovementPoint lastPoint = startingPoint;
        for (int i = 0; i < segments.length; i++){
            System.out.println("Starting to process segment " + (i + 1) + " / " + segments.length);
            Segment segment = segments[i];
            if (segment instanceof WaitSegment){
                WaitSegment waitSegment = (WaitSegment) segment;
                movementMap.addWaitPeriod(waitSegment.getPosition(0),waitSegment.getDuration());
            }else{
                // We can skip the check for if there is going to be another segment after current because PathBuilder ensures that all valid paths end with an end (wait) segment.
                boolean shouldComeToStop = segments[i+1] instanceof WaitSegment;
                //
//                movementMap = MovementMap.combine(movementMap, renderMovementMapThroughTime(compileVelocityProfile(segments[i], lastPoint, sampleRate, shouldComeToStop),sampleRate));
                MovementMap nextMap = renderMovementMapThroughTime(compileVelocityProfile(segments[i], lastPoint, sampleRate, shouldComeToStop),sampleRate);
                movementMap.addMovementPoints(nextMap.getAllPoints());
                lastPoint = movementMap.getAllPoints().get(movementMap.getAllPoints().size()-1);
            }
            System.out.println("Processed segment " + (i + 1) + " / " + segments.length);
        }
        return movementMap;
    }
    private MovementMap compileVelocityProfile(Segment segment, MovementPoint startingPoint, double sampleRate, boolean comeToStop) {
        //Algorithm steps
        // 1. convert to be in terms of distance - DONE
        // 2. get the max points of curvature.
        // 3. Use the curve of those points to determine the max robot velocity at that curve ( and therefore the slowest theoretically in the spline the robot could move at)
        // 4. Simulate at a sample rate of distance, the fastest the robot would be able to accelerate from each of those distances
        // 5. Compute full velocity profile by comparing each simulation and taking the lowest velocity at each point
        // 7. return that MovementMap which is the velocity profile.
        DistanceMap distanceMap = new DistanceMap(segment);
        double startingDistance = 0;
        System.out.println("Starting Ideal Map Generation");
        double endingDistance = distanceMap.getMaxDistance();
        MovementMap idealMap = calculateIdealMovementMap(distanceMap, segment.getSpeedRate(),sampleRate);
        System.out.println("Finished Ideal Map Generation, starting curvature analysis");
        // TODO - make new method to get distances at the maxCurvature points
        double[] maxCurvature = distanceMap.getSegmentMaxCurvaturePoints();
        double[] simulationPoints = new double[]{
                startingDistance,
                // TODO - use distance at these points, not the curvature values themselves
                maxCurvature[0],
                maxCurvature[1],
                endingDistance
        };
        ArrayList<VelocityMap> simulations = new ArrayList<>();
        for (double point : simulationPoints){
            System.out.println("Starting simulation at distance " + point + " / " + endingDistance);
            double currentDistance = point;
            VelocityMap currentProfile = new VelocityMap(point, sampleRate, startingPoint, idealMap.getPoint(endingDistance));

            MovementPoint lastPoint;
            if (point == simulationPoints[0]){
                lastPoint = startingPoint;
            }else{
                lastPoint = idealMap.getPoint(point);
            }
            while (currentDistance <= endingDistance){
                MovementPoint basicResult;
                if (comeToStop && currentDistance == endingDistance){
                    basicResult = simulateStep(sampleRate,currentDistance,distanceMap, new MovementPoint(distanceMap.getPositionAtDistance(currentDistance),0,0,0,0,0,0), idealMap);
                }else{
                    basicResult = simulateStep(sampleRate,currentDistance,distanceMap, lastPoint, idealMap);
                }
                currentDistance += Coordinate.getDistanceBetweenCoordinates(basicResult.getPosition(), lastPoint.getPosition());
                currentProfile.addForwardPoint(lastPoint);
                lastPoint = basicResult;
            }
            System.out.println("Finished forward simulation");
            // backward pass
            currentDistance = point-sampleRate;
            while (currentDistance >= startingDistance){
                MovementPoint basicResult;
                if (comeToStop && currentDistance == endingDistance){
                    basicResult = simulateStep(-sampleRate,currentDistance,distanceMap, new MovementPoint(distanceMap.getPositionAtDistance(currentDistance),0,0,0,0,0,0), idealMap);
                }else{
                    basicResult = simulateStep(-sampleRate,currentDistance,distanceMap, lastPoint, idealMap);
                }

                currentDistance -= Coordinate.getDistanceBetweenCoordinates(basicResult.getPosition(), lastPoint.getPosition());
                currentProfile.addBackPassPoint(lastPoint);
                lastPoint = basicResult;
            }
            System.out.println("Finished backward simulation");
            simulations.add(currentProfile);
        }
        System.out.println("Finished all simulations");
        MovementMap finalizedVelocityMap = VelocityMap.generateMovementFromVelocityMaps(simulations.toArray(new VelocityMap[0]), sampleRate);
        return finalizedVelocityMap;
    }

    /**
     * Renders a finalized distance-indexed velocity profile into a time-indexed MovementMap.
     * Unlike {@link #simulateStep}, this does not recompute or clamp velocities against an
     * ideal profile — it simply integrates the acceleration already stored on each point
     * (derived from robot movement parameters during the original simulation passes) forward
     * through time, producing continuous position and velocity samples.
     * @param velocityProfile the finalized MovementMap indexed by distance
     * @param timeSampleRate the time step, in seconds, of the resulting profile
     * @return a MovementMap whose unit is time (seconds) instead of distance
     */
    private MovementMap renderMovementMapThroughTime(MovementMap velocityProfile, double timeSampleRate){
        ArrayList<MovementPoint> distancePoints = velocityProfile.getAllPoints();
        if (distancePoints.isEmpty()) throw new IllegalArgumentException("distanceProfile is empty");

        double maxDistance = velocityProfile.getSampleRate() * (distancePoints.size() - 1);
        MovementMap timeProfile = new MovementMap(timeSampleRate);

        MovementPoint currentPoint = distancePoints.get(0);
        double traveledDistance = 0.0;
        timeProfile.addMovementPoint(currentPoint);

        while (traveledDistance < maxDistance){
            // TODO - Don't simulate, just LERP. Simulation can get stuck if vel and acc are zero.
            MovementPoint nextPoint = simulateRenderStep(timeSampleRate, currentPoint);
            traveledDistance += Coordinate.getDistanceBetweenCoordinates(currentPoint.getPosition(), nextPoint.getPosition());
            // Pull the finalized acceleration for the newly reached distance so the render
            // reflects how the path's acceleration profile changes as distance accumulates.
            MovementPoint distanceLookup = velocityProfile.getPoint(Math.min(traveledDistance, maxDistance));
            nextPoint.setAcceleration(distanceLookup.getAccelX(), distanceLookup.getAccelY(), distanceLookup.getAccelAngle());
            timeProfile.addMovementPoint(nextPoint);
            currentPoint = nextPoint;
        }
        return timeProfile;
    }

    /**
     * Advances a MovementPoint forward by dt using its already-known acceleration (derived
     * from robot movement parameters during the original simulation pass in {@link #simulateStep}).
     * Does not recompute or clamp velocity against an ideal profile — just renders continuous
     * position and velocity through time.
     */
    private MovementPoint simulateRenderStep(double dt, MovementPoint point){
        double velX = point.getVelX() + point.getAccelX() * dt;
        double velY = point.getVelY() + point.getAccelY() * dt;
        double velAngle = point.getVelAngle() + point.getAccelAngle() * dt;

        double posX = point.getPosition().getX() + point.getVelX() * dt + 0.5 * point.getAccelX() * dt * dt;
        double posY = point.getPosition().getY() + point.getVelY() * dt + 0.5 * point.getAccelY() * dt * dt;
        double posAngle = point.getPosition().getAngle() + point.getVelAngle() * dt + 0.5 * point.getAccelAngle() * dt * dt;

        return new MovementPoint(
            new Coordinate(posX, posY, posAngle),
            velX, velY, velAngle,
            point.getAccelX(), point.getAccelY(), point.getAccelAngle()
        );
    }
    private MovementMap calculateIdealMovementMap(DistanceMap distanceMap, double segmentSpeedRatio, double sampleRate){
        MovementMap resultingIdealMap = new MovementMap(sampleRate);
        double startDistance = distanceMap.getMinDistance();
        double endDistance = distanceMap.getMaxDistance();
        Coordinate lastPoint = distanceMap.getPositionAtDistance(startDistance);
        // TODO - Consider that distanceMap only has 100 samples, but this may loop many times more than that (eg. 4k for 40 inches)
        for (double p = startDistance + sampleRate; p < endDistance; p += sampleRate) {
            Coordinate newPoint = distanceMap.getPositionAtDistance(p);
            double headingDegrees = getHeadingToCoordinate(lastPoint, newPoint);
            double curvature = distanceMap.getCurvatureAtDistance(p);
            double maxVelocity = movementParameters.getMaxVelocity(headingDegrees, newPoint.getAngle() - lastPoint.getAngle()) * segmentSpeedRatio;
            // TODO - DO something better here. Something like "if the curvature prevents robot from traveling at this speed, then reduce the speed to what can be done at this curvature"
//            if (curvatureEffect / curvature < 1)
//                maxVelocity *= curvatureEffect / curvature;
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
            lastPoint = newPoint;
        }
        return resultingIdealMap;
    }
    // Unused for now, may be helpful when we test as an iteration.
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
        double finalX = point.getPosition().getX() + step * (point.getVelX() + (1.0/2.0) * deltaVelocityX);
        double finalY = point.getPosition().getY() + step * (point.getVelY() + (1.0/2.0) * deltaVelocityY);
        double finalAccelerationX = deltaVelocityX / absStep;
        double finalAccelerationY = deltaVelocityY / absStep;
        
        // ==== ROTATIONAL VELOCITY CALCULATIONS ====
        // Get current angular velocity and calculate angular acceleration
        double currentAngularVelocity = point.getVelAngle();
        double angularAcceleration = movementParameters.getMaxStableAngularAcceleration(currentAngularVelocity, idealMap.getPoint(newDistance).getVelAngle()>=0);
        
        // Apply angular acceleration over the time step
        double finalVelocityAngle = currentAngularVelocity + (angularAcceleration * step);
        
        // Clamp angular velocity to ideal profile using magnitude comparison for bidirectional motion
        if (Math.abs(idealMap.getPoint(newDistance).getVelAngle()) < Math.abs(finalVelocityAngle)){
            finalVelocityAngle = idealMap.getPoint(newDistance).getVelAngle();
        }
        //TODO: Store time information from this simulation, and then lerp between the time steps
        // Calculate angular displacement using average angular velocity
        double deltaVelocityAngle = finalVelocityAngle - currentAngularVelocity;
        double finalAngle = point.getPosition().getAngle() + step * (currentAngularVelocity + (1.0/2.0) * deltaVelocityAngle);
        double finalAccelerationAngle = deltaVelocityAngle / absStep;
        
        // ==== CREATE NEW MOVEMENT POINT ====
        MovementPoint newPoint = new MovementPoint(
                new Coordinate(finalX, finalY, finalAngle),
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
    private double getWorldHeadingToCoordinate(Coordinate start, Coordinate end){
        double xDifference = end.getX() - start.getX();
        double yDifference = end.getY() - start.getY();
        return Math.toDegrees(Math.atan2(yDifference, xDifference));
    }
    private double getHeadingToCoordinate(Coordinate start, Coordinate end){
        double xDifference = end.getX() - start.getX();
        double yDifference = end.getY() - start.getY();
        double baseAngle = Math.atan2(yDifference, xDifference);
        baseAngle *= (180.0/Math.PI); // convert to degrees from RADS
        double result = baseAngle - start.getAngle();
        while (result >= 180) result -= 360;
        while (result < -180) result += 360;
        return result;
    }

}
