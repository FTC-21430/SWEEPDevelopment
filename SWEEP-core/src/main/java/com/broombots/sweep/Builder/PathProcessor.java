package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LerpFunction;
import com.broombots.sweep.Classes.LerpFunctionBuilder;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.Segments.WaitSegment;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Objects;

public class PathProcessor {
    private RobotMovementParameters movementParameters;
    private Pos2D lastRenderedPosition = null;
    LerpFunction regression;
    ArrayList<Double> speedRatios = new ArrayList<>();

    public PathProcessor(RobotMovementParameters movementParameters) {
        // Initialize the motion profile processor with the given movement parameters
        this.movementParameters = movementParameters;

        final double pi = Math.PI; // shorthand for myself because I need to type this a lot, Style guide exception please.

        // Create regression from change in RAD per inch to speed change //TODO TUNE and change to be config in movementParameters
        regression = new LerpFunctionBuilder()
                .addPoint(0,1)
                .addPoint(0.03, 0.98)
                .addPoint(0.04,0.8)
                .addPoint(0.35, 0.05)
                .build();
    }
    public ArrayList<Object> processPath(Segment[] segments, double sampleRate, PathPoint startingPoint, double endingSpeedRatio) {
        ArrayList<Double> segmentTimeMarkers = new ArrayList<>();
        ArrayList<PathPoint> path = new ArrayList<>();
        ArrayList<ArrayList<Segment>> brokenSegments = breakUpComplexRoute(segments);
        for (int i = 0; i < brokenSegments.size(); i++){
            ArrayList<Segment> segmentList = brokenSegments.get(i);
            if (segmentList.get(0) instanceof WaitSegment){
                path.addAll(processWaitSegments(segmentList.toArray(new Segment[0]), sampleRate, path.isEmpty() ? startingPoint.time : path.get(path.size()-1).time, segmentTimeMarkers));
            }else{
                double speedRate = i < brokenSegments.size()-1? 0 : endingSpeedRatio;
                PathPoint prevPoint = i > 0? path.get(path.size()-1) : startingPoint;
                path.addAll(processMovementSegments(segmentList.toArray(new Segment[0]), sampleRate, prevPoint, speedRate, segmentTimeMarkers));
            }
        }
        ArrayList<Object> result = new ArrayList<>();
        result.add(path);
        result.add(segmentTimeMarkers);
        return result;
    }
    private ArrayList<ArrayList<Segment>> breakUpComplexRoute(Segment[] segments){
        ArrayList<ArrayList<Segment>> result = new ArrayList<>();
        ArrayList<Segment> currentSegmentList = new ArrayList<>();
        for (Segment segment : segments){
            if (segment instanceof WaitSegment){
                if (!currentSegmentList.isEmpty()) result.add(currentSegmentList);
                ArrayList<Segment> waitSegmentList = new ArrayList<>();
                waitSegmentList.add(segment);
                result.add(waitSegmentList);
                currentSegmentList = new ArrayList<>();
            }else{
                currentSegmentList.add(segment);
            }
        }
        if (!currentSegmentList.isEmpty()) result.add(currentSegmentList);
        return result;
    }
    private ArrayList<PathPoint> processWaitSegments(Segment[] segments, double sampleRate, double startTime, ArrayList<Double> segmentTimeMarkers){
        ArrayList<PathPoint> path = new ArrayList<>();
        double lastTime = startTime;
        for (Segment segment : segments){
            segmentTimeMarkers.add(lastTime);
            WaitSegment waitSegment = (WaitSegment) segment;
            for (double t = lastTime; t < waitSegment.getDuration()+lastTime; t += sampleRate){
                Pos2D pos2D = waitSegment.getPosition(t);
                if (Double.isNaN(pos2D.angle)){
                    pos2D.angle = lastRenderedPosition.angle;
                }
                PathPoint point = new PathPoint();
                point.position = pos2D;
                point.velocity = new Pos2D(0,0,0);
                point.time = t;
                path.add(point);
            }
            lastTime = path.get(path.size()-1).time;
        }
        lastRenderedPosition = path.get(path.size()-1).position;
        return path;
    }
    private ArrayList<PathPoint> processMovementSegments(Segment[] segments, double sampleRate, PathPoint startingPoint, double endingSpeedRatio, ArrayList<Double> segmentTimeMarkers){
        ArrayList<PathPoint> path = new ArrayList<>();
        ArrayList<Integer> waypointIndexInPath = new ArrayList<>();

        PathPoint stPoint = startingPoint;

        if (Double.isNaN(stPoint.position.angle)){
            stPoint.position = lastRenderedPosition;
        }

        path.add(stPoint);
        speedRatios.add(segments[0].getSpeedRate());
        for (int i = 0; i < segments.length; i++){
            Segment segment = segments[i];
            waypointIndexInPath.add(path.size());
            for (double t = 0.0 + 1e-7; t < 1.0; t += sampleRate){
                Pos2D pos2D = segment.getPosition(t);
                PathPoint point = new PathPoint();
                point.position = pos2D;
                path.add(point);
                speedRatios.add(segment.getSpeedRate());
            }

        }
        waypointIndexInPath.add(path.size());
        PathPoint endPoint = new PathPoint();
        endPoint.position = segments[segments.length-1].getPosition(1);


        double endAngle = Pos2D.getMovementDirectionRad(path.get(path.size() - 1).position, endPoint.position);
        double terminalSpeed = movementParameters.getMaxVelocity(Math.toRadians(endAngle)) * Math.max(0.0, Math.min(1.0, endingSpeedRatio));
        endPoint.velocity = new Pos2D(
                terminalSpeed * Math.cos(endAngle),
                terminalSpeed * Math.sin(endAngle),
                0
        );
        path.add(endPoint);

        path = generatePathPoints(path);

        double time = startingPoint.time;
        segmentTimeMarkers.add(time);

        for (int i = 1; i < path.size(); i++){
            PathPoint lastPoint = path.get(i-1);
            PathPoint currentPoint = path.get(i);

            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double avgVelocity = (lastPoint.velocity.getMagnitude()+currentPoint.velocity.getMagnitude())/2.0;
            double dt;
            if (avgVelocity > 1e-5) {
                dt = distance / avgVelocity;
            }else{
                dt = 1e-5;
            }

            time += dt;
            if (waypointIndexInPath.contains(i)){
                segmentTimeMarkers.add(time);
            }


            path.get(i).time = time;
        }

        speedRatios.clear();
        lastRenderedPosition = path.get(path.size()-1).position;
        return path;
    }

    private ArrayList<PathPoint> generatePathPoints(ArrayList<PathPoint> path) {
        // forward pass for acceleration and primary velocity limits.
        for (int i = 1; i < path.size()-2; i++){

            PathPoint lastPoint = path.get(i-1);
            PathPoint currentPoint = path.get(i);
            PathPoint nextPoint = null;
            if (i < path.size()-2){
                nextPoint = path.get(i+1);
            }
            double d1 = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);

            double adjustedSpeed = calculateSpeedGoal(lastPoint,currentPoint,nextPoint, i);
            // calculate new moment of velocity

            double lastVelocityScalar = Math.hypot(lastPoint.velocity.x, lastPoint.velocity.y);
            double momentAcceleration = movementParameters.getMaxStableAcceleration(Math.toDegrees(Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position)));
            double newVelocityScalar = calculateNewVelocity(d1, momentAcceleration, lastVelocityScalar);
            if (newVelocityScalar > adjustedSpeed) newVelocityScalar = adjustedSpeed;
            double angleRad = Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position);
            double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);
//
            double angularDifference = wrapDegrees(currentPoint.position.angle - lastPoint.position.angle);
            double angularSign = Math.signum(angularDifference);
            double angularVelocity = calculateNewVelocity(Math.abs(angularDifference), movementParameters.getMaxStableAngularAcceleration(newVelocityScalar, angularSign > 0), Math.abs(lastPoint.velocity.angle));
            angularVelocity *= angularSign;
            if (Math.abs(angularVelocity) > movementParameters.getMaxAngleVelocity()) angularVelocity = movementParameters.getMaxAngleVelocity() * angularSign;
            if (i % 25 == 0){
                System.out.println("angularVelocity = " + angularVelocity);
            }
            Pos2D velocity = new Pos2D(xVelocity,yVelocity,0);
            path.get(i).velocity = velocity;


        }

        System.out.println("Finished Front Pass");
        // back pass for de-accel period
        for (int i = path.size()-2; i > 1; i--){
            PathPoint lastPoint = path.get(i+1);
            PathPoint currentPoint = path.get(i);

            if (currentPoint.velocity == null) currentPoint.velocity = new Pos2D(0,0,0);
            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double lastVelocityScalar = Math.hypot(lastPoint.velocity.x, lastPoint.velocity.y);

            double forwardPassVelocityScalar = Math.hypot(currentPoint.velocity.x, currentPoint.velocity.y);
            double newVelocityScalar = calculateNewVelocity(distance, movementParameters.getMaxStableAcceleration(Math.toDegrees(Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position))), lastVelocityScalar);
            if (forwardPassVelocityScalar < newVelocityScalar && i != path.size()-2) continue; // respect the forward pass output
            // point order is opposite from forward pass because we are going backwards.
            double angleRad = Pos2D.getMovementDirectionRad(lastPoint.position, currentPoint.position);double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);

            double angularDifference = wrapDegrees(currentPoint.position.angle - lastPoint.position.angle);
            double angularSign = Math.signum(angularDifference);
            double angularVelocity = calculateNewVelocity(Math.abs(angularDifference), movementParameters.getMaxStableAngularAcceleration(Math.hypot(currentPoint.velocity.x, currentPoint.velocity.y), angularSign > 0), Math.abs(lastPoint.velocity.angle));
            angularVelocity *= angularSign;
            if (Math.abs(angularVelocity) > movementParameters.getMaxAngleVelocity()) angularVelocity = movementParameters.getMaxAngleVelocity() * angularSign;

            Pos2D velocity = new Pos2D(xVelocity,yVelocity,0);
            currentPoint.velocity = velocity;
        }
        System.out.println("Finished Back Pass");
        return path;
    }

    private double calculateNewVelocity(double dTraveled, double acceleration, double lastVelocity){
        return Math.sqrt(Math.pow(lastVelocity, 2) + 2 * (acceleration * dTraveled));
    }

    private double calculateSpeedGoal(PathPoint lastPoint, PathPoint currentPoint, PathPoint nextPoint, int idx){
        // find curvature in rad per inch
        double d1 = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
        double d2;
        double angle1 = Pos2D.getMovementDirectionRad(lastPoint.position,currentPoint.position);
        double angle2;
        if (nextPoint != null){
            d2 = Pos2D.getDistanceBetweenCoordinates(currentPoint.position, nextPoint.position);
            angle2 = Pos2D.getMovementDirectionRad(currentPoint.position,nextPoint.position);
        }else {
            d2 = d1;
            angle2 = angle1;
        }
        double averageTravelDistance = Math.abs((d1+d2)/2.0);
        double changeInAngleRAD = Math.abs(wrapRadians(angle2 - angle1));
        double deltaRadPerInch;
        if (changeInAngleRAD < 1e-8 || averageTravelDistance < 1e-3){
            deltaRadPerInch = 0;
        }
        else{
            deltaRadPerInch = changeInAngleRAD/averageTravelDistance;
        }
        return movementParameters.getMaxVelocity(Math.toDegrees(Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position))) * regression.getValue(deltaRadPerInch) * speedRatios.get(idx);
    }
    private double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
    private double wrapDegrees(double angle){
        double resultingAngle = angle;
        while (resultingAngle > 180) resultingAngle -= 360;
        while (resultingAngle < -180) resultingAngle += 360;
        return resultingAngle;
    }

}
