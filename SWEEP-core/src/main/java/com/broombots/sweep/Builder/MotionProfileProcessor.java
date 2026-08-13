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
                DistanceMap distanceMap = new DistanceMap(segment);
                MovementMap nextMap = renderMovementMapThroughTime(compileVelocityProfile(segments[i], lastPoint, sampleRate, shouldComeToStop, distanceMap), distanceMap ,sampleRate);
                movementMap.addMovementPoints(nextMap.getAllPoints());
                lastPoint = movementMap.getAllPoints().get(movementMap.getAllPoints().size()-1);
            }
            System.out.println("Processed segment " + (i + 1) + " / " + segments.length);
        }
        return movementMap;
    }
    private MovementMap compileVelocityProfile(Segment segment, MovementPoint startingPoint, double sampleRate, boolean comeToStop, DistanceMap distanceMap) {
        //Algorithm steps
        // 1. convert to be in terms of distance - DONE
        // 2. get the max points of curvature.
        // 3. Use the curve of those points to determine the max robot velocity at that curve ( and therefore the slowest theoretically in the spline the robot could move at)
        // 4. Simulate at a sample rate of distance, the fastest the robot would be able to accelerate from each of those distances
        // 5. Compute full velocity profile by comparing each simulation and taking the lowest velocity at each point
        // 7. return that MovementMap which is the velocity profile.

        double startingDistance = 0;
        System.out.println("Starting Ideal Map Generation");
        double endingDistance = distanceMap.getMaxDistance();
        MovementMap idealMap = calculateIdealMovementMap(distanceMap, segment.getSpeedRate(),sampleRate);
        System.out.println("Finished Ideal Map Generation, starting curvature analysis");
        ArrayList<Double> maxCurvature = distanceMap.getSegmentDistancesWithLocalMaximaCurvature();
        double[] simulationPoints = new double[]{
                startingDistance,
                maxCurvature.get(0),
                maxCurvature.get(1),
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
                if (comeToStop && Math.abs(currentDistance - endingDistance) <= 1e-9){
                    System.out.println("STOPPING POINT!");
                    basicResult = simulateStep(sampleRate,currentDistance,distanceMap, new MovementPoint(distanceMap.getPositionAtDistance(currentDistance),0,0,0,0,0,0), idealMap);
                }else{
                    basicResult = simulateStep(sampleRate,currentDistance,distanceMap, lastPoint, idealMap);
                }
                currentDistance += Coordinate.getDistanceBetweenCoordinates(basicResult.getPosition(), lastPoint.getPosition());
                currentProfile.addForwardPoint(basicResult);
                lastPoint = basicResult;
            }
            System.out.println("Finished forward simulation");
            // backward pass

            currentDistance = point;
            if (comeToStop && Math.abs(currentDistance - endingDistance) <= 1e-9){
                lastPoint = new MovementPoint(distanceMap.getPositionAtDistance(currentDistance),0,0,0,0,0,0);
            }else{
                lastPoint = idealMap.getPoint(point);
            }
            while (currentDistance > startingDistance+1e-9){
                MovementPoint basicResult;

                if (comeToStop && Math.abs(currentDistance - endingDistance) <= 1e-9){
                    basicResult = simulateStep(-sampleRate,currentDistance,distanceMap, new MovementPoint(distanceMap.getPositionAtDistance(currentDistance),0,0,0,0,0,0), idealMap);
                }else{
                    basicResult = simulateStep(-sampleRate,currentDistance,distanceMap, lastPoint, idealMap);
                }
                currentDistance -= Coordinate.getDistanceBetweenCoordinates(basicResult.getPosition(), lastPoint.getPosition());
                currentProfile.addBackPassPoint(basicResult);

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
*    * @param spline the distanceMap that holds the full spline, ensuring that our results include the positions they are supposed to be at.
     * @param sampleRate the time step, in seconds, of the resulting profile
     * @return a MovementMap whose unit is time (seconds) instead of distance
     */
    private MovementMap renderMovementMapThroughTime(MovementMap velocityProfile, DistanceMap spline, double sampleRate){
        ArrayList<MovementPoint> distancePoints = velocityProfile.getAllPoints();
        if (distancePoints.isEmpty()) throw new IllegalArgumentException("distanceProfile is empty");

        double maxDistance = velocityProfile.getMaxKey();
        TimeProfile timeProfile = new TimeProfile();

        MovementPoint firstPoint = distancePoints.get(0);
        MovementPoint currentPoint = new MovementPoint(
                spline.getPositionAtDistance(spline.getMinDistance()),
                firstPoint.getVelX(),
                firstPoint.getVelY(),
                firstPoint.getVelAngle(),
                firstPoint.getAccelX(),
                firstPoint.getAccelY(),
                firstPoint.getAccelAngle()
        );
        double traveledDistance = 0.0;
        double time = 0.0;
        timeProfile.addPoint(currentPoint, time);

        while (traveledDistance < maxDistance + 1e-5){
            traveledDistance += sampleRate;
            MovementPoint distanceLookup = velocityProfile.getPoint(Math.min(traveledDistance, maxDistance));
            double velX = Math.abs(distanceLookup.getVelX()) < 1e-5? 0: distanceLookup.getVelX();
            double velY = Math.abs(distanceLookup.getVelY()) < 1e-5? 0: distanceLookup.getVelY();
            double velAngle = Math.abs(distanceLookup.getVelAngle()) < 1e-5? 0: distanceLookup.getVelAngle();
            double accelX = Math.abs(distanceLookup.getAccelX()) < 1e-5? 0: distanceLookup.getAccelX();
            double accelY = Math.abs(distanceLookup.getAccelY()) < 1e-5? 0: distanceLookup.getAccelY();
            double accelAngle = Math.abs(distanceLookup.getAccelAngle()) < 1e-5? 0: distanceLookup.getAccelAngle();
            MovementPoint finalPoint = new MovementPoint(
                    spline.getPositionAtDistance(traveledDistance),
                    velX,
                    velY,
                    velAngle,
                    accelX,
                    accelY,
                    accelAngle
            );
            distanceLookup = null; // free the memory this is using up
            double dt = getTimeBetweenPoints(currentPoint, finalPoint);
            time += dt;
            timeProfile.addPoint(finalPoint, time);
            currentPoint = finalPoint;
        }
        return timeProfile.normalizeTimeProfile(sampleRate);
    }
    private double getTimeBetweenPoints(MovementPoint start, MovementPoint end){
        double distance = Coordinate.getDistanceBetweenCoordinates(start.getPosition(), end.getPosition());
        double avgVelocityMagnitude = (start.getVelocityMagnitude() + end.getVelocityMagnitude())/2;
        return distance / avgVelocityMagnitude;
    }
    private MovementMap calculateIdealMovementMap(DistanceMap distanceMap, double segmentSpeedRatio, double sampleRate){
        MovementMap resultingIdealMap = new MovementMap(sampleRate);
        double startDistance = distanceMap.getMinDistance();
        double endDistance = distanceMap.getMaxDistance();
        Coordinate lastPoint = distanceMap.getPositionAtDistance(startDistance);
        // TODO - Consider that distanceMap only has 100 samples, but this may loop many times more than that (eg. 4k for 40 inches)
        for (double p = startDistance + sampleRate*100; p < endDistance; p += sampleRate*100) {
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
        // TODO: get the rotation error first?
        double tempAcceleration = movementParameters.getMaxStableAcceleration(heading, 0);
        
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
        double finalX = point.getPosition().getX() + step * (point.getVelX() + (0.5) * deltaVelocityX);
        double finalY = point.getPosition().getY() + step * (point.getVelY() + (0.5) * deltaVelocityY);
        double finalAccelerationX = deltaVelocityX / absStep;
        double finalAccelerationY = deltaVelocityY / absStep;
        
        // ==== ROTATIONAL VELOCITY CALCULATIONS ====
        // Get current angular velocity and calculate angular acceleration
        double currentAngularVelocity = point.getVelAngle();
        double angularAcceleration = movementParameters.getMaxStableAngularAcceleration(finalVelocityMagnitude, true);
        angularAcceleration *= Math.min((idealMap.getPoint(newDistance).getPosition().getAngle() - point.getPosition().getAngle()) / movementParameters.getAngleFullPowerToErrorThreshold(), 1);
        
        // Apply angular acceleration over the time step
        double finalVelocityAngle = currentAngularVelocity + (angularAcceleration * step);
        
        // Clamp angular velocity to ideal profile
        if (Math.abs(idealMap.getPoint(newDistance).getVelAngle()) < Math.abs(finalVelocityAngle)){
            finalVelocityAngle = idealMap.getPoint(newDistance).getVelAngle();
        }
        //TODO: Store time information from this simulation, and then lerp between the time steps
        // Calculate angular displacement using average angular velocity - Update, will not work because of the position discontinuity in the velocity profile

        double finalAngle = point.getPosition().getAngle() + step * (currentAngularVelocity + (1.0/2.0) * angularAcceleration * step);
        
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
        point.setAcceleration(finalAccelerationX, finalAccelerationY, angularAcceleration);
        return newPoint;
    }
    private double getGlobalHeadingToCoordinate(Coordinate start, Coordinate end){
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
