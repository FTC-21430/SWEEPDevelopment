package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LerpFunction;
import com.broombots.sweep.Classes.LerpFunctionBuilder;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;

import java.nio.file.Path;
import java.util.ArrayList;

public class PathProcessor {
    private RobotMovementParameters movementParameters;

    private double acceleration = 22; // inches per sec^2
    private double angularAcceleration = 60; // degrees per sec^2
    private double maxVelocity = 48;
    private double maxAngularVelocity = 120; // degrees per sec
    LerpFunction regression;
    ArrayList<Double> speedRatios = new ArrayList<>();

    public PathProcessor(RobotMovementParameters movementParameters) {
        // Initialize the motion profile processor with the given movement parameters
        this.movementParameters = movementParameters;

        final double pi = Math.PI; // shorthand for myself because I need to type this a lot, Style guide exception please.

        // Create regression from change in RAD per inch to speed change //TODO TUNE
        regression = new LerpFunctionBuilder()
                .addPoint(0,1)
                .addPoint(0.03, 0.98)
                .addPoint(0.04,0.8)
                .addPoint(0.35, 0.05)
                .build();
    }
    public ArrayList<PathPoint> processPath(Segment[] segments, double sampleRate, PathPoint startingPoint, double endingSpeedRatio) {
        ArrayList<PathPoint> path = new ArrayList<>();

        PathPoint stPoint = startingPoint;
        path.add(stPoint);
        speedRatios.add(segments[0].getSpeedRate());
        for (int i = 0; i < segments.length; i++){
            Segment segment = segments[i];
            for (double t = 0.0 + 1e-7; t < 1.0; t += sampleRate){
                Pos2D pos2D = segment.getPosition(t);
                PathPoint point = new PathPoint();
                point.position = pos2D;
                path.add(point);
                speedRatios.add(segment.getSpeedRate());
            }
        }
        PathPoint endPoint = new PathPoint();
        endPoint.position = segments[segments.length-1].getPosition(1);

        double terminalSpeed = maxVelocity * Math.max(0.0, Math.min(1.0, endingSpeedRatio));
        double endAngle = Pos2D.getMovementDirectionRad(path.get(path.size() - 1).position, endPoint.position);
        endPoint.velocity = new Pos2D(
                terminalSpeed * Math.cos(endAngle),
                terminalSpeed * Math.sin(endAngle),
                0
        );
        path.add(endPoint);

        path = generatePathPoints(path);

        double time = startingPoint.time;

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

            path.get(i).time = time;
        }
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
            double momentAcceleration = acceleration;
            double newVelocityScalar = calculateNewVelocity(d1, momentAcceleration, lastVelocityScalar);
            if (newVelocityScalar > adjustedSpeed) newVelocityScalar = adjustedSpeed;
            double angleRad = Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position);
            double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);
//
            double angularDifference = wrapDegrees(currentPoint.position.angle - lastPoint.position.angle);
            double angularSign = Math.signum(angularDifference);
            double angularVelocity = calculateNewVelocity(Math.abs(angularDifference), angularAcceleration, Math.abs(lastPoint.velocity.angle));
            angularVelocity *= angularSign;
            if (Math.abs(angularVelocity) > maxAngularVelocity) angularVelocity = maxAngularVelocity * angularSign;
            if (i % 25 == 0){
                System.out.println("angularVelocity = " + angularVelocity);
            }
            Pos2D velocity = new Pos2D(xVelocity,yVelocity,10);
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
            double newVelocityScalar = calculateNewVelocity(distance, acceleration, lastVelocityScalar);
            if (forwardPassVelocityScalar < newVelocityScalar && i != path.size()-2) continue; // respect the forward pass output
            // point order is opposite from forward pass because we are going backwards.
            double angleRad = Pos2D.getMovementDirectionRad(lastPoint.position, currentPoint.position);double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);

            double angularDifference = wrapDegrees(currentPoint.position.angle - lastPoint.position.angle);
            double angularSign = Math.signum(angularDifference);
            double angularVelocity = calculateNewVelocity(Math.abs(angularDifference), angularAcceleration, Math.abs(lastPoint.velocity.angle));
            angularVelocity *= angularSign;
            if (Math.abs(angularVelocity) > maxAngularVelocity) angularVelocity = maxAngularVelocity * angularSign;

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
        return maxVelocity * regression.getValue(deltaRadPerInch) * speedRatios.get(idx);
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
