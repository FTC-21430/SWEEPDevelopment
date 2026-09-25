package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;

import java.nio.file.Path;
import java.util.ArrayList;

public class PathProcessor {
    private RobotMovementParameters movementParameters;

    private double acceleration = 12; // inches per sec^2
    private double angularAcceleration = 60; // degrees per sec^2
    private double maxVelocity = 80;
    public PathProcessor(RobotMovementParameters movementParameters) {
        // Initialize the motion profile processor with the given movement parameters
        this.movementParameters = movementParameters;
    }
    public ArrayList<PathPoint> processPath(Segment[] segments, double sampleRate, PathPoint startingPoint) {
        ArrayList<PathPoint> path = new ArrayList<>();
        PathPoint stPoint = new PathPoint();
        stPoint.position = segments[0].getPosition(0);
        stPoint.velocity = new Pos2D(0,0,0);
        stPoint.time = 0;
        path.add(stPoint);
        for (int i = 0; i < segments.length; i++){
            Segment segment = segments[i];
            for (double t = 0.0 + 1e-7; t < 1.0; t += sampleRate){
                Pos2D pos2D = segment.getPosition(t);
                PathPoint point = new PathPoint();
                point.position = pos2D;
                path.add(point);
            }
        }
        PathPoint endPoint = new PathPoint();
        endPoint.position = segments[segments.length-1].getPosition(1);
        endPoint.velocity = new Pos2D(0,0,0);
        path.add(endPoint);

        path = generatePathPoints(path);

        double time = startingPoint.time;

        for (int i = 1; i < path.size()-1; i++){
            PathPoint lastPoint = path.get(i-1);
            PathPoint currentPoint = path.get(i);

            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double avgVelocity = (lastPoint.velocity.getMagnitude()+currentPoint.velocity.getMagnitude())/2.0;

            double dt = distance/avgVelocity;
            time += dt;

            currentPoint.time = time;
        }

        return path;
    }

    private ArrayList<PathPoint> generatePathPoints(ArrayList<PathPoint> path) {
        // forward pass for acceleration and primary velocity limits.
        for (int i = 1; i < path.size()-2; i++){
            PathPoint lastPoint = path.get(i-1);
            PathPoint currentPoint = path.get(i);

            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double lastVelocityScalar = Math.hypot(lastPoint.velocity.x, lastPoint.velocity.y);

            double newVelocityScalar = calculateNewVelocity(distance, acceleration, lastVelocityScalar);
            if (newVelocityScalar > maxVelocity) newVelocityScalar = maxVelocity;
            double angleRad = Math.atan2(currentPoint.position.y - lastPoint.position.y, currentPoint.position.x - lastPoint.position.x);

            double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);

//            double angularDifference = currentPoint.position.angle - lastPoint.position.angle;
//            double angularVelocity = calculateNewVelocity(angularDifference, angularAcceleration, lastPoint.velocity.angle);
            // TODO: handle angular velocity after x,y

            Pos2D velocity = new Pos2D(xVelocity,yVelocity,0);
            currentPoint.velocity = velocity;
        }
        // back pass for de-accel period
        for (int i = path.size()-2; i > 1; i--){
            PathPoint lastPoint = path.get(i+1);
            PathPoint currentPoint = path.get(i);

            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double lastVelocityScalar = Math.hypot(lastPoint.velocity.x, lastPoint.velocity.y);

            double forwardPassVelocityScalar = Math.hypot(currentPoint.velocity.x, currentPoint.velocity.y);
            double newVelocityScalar = calculateNewVelocity(distance, acceleration, lastVelocityScalar);
            if (forwardPassVelocityScalar < newVelocityScalar) break; // respect the forward pass output

            double angleRad = Math.atan2(currentPoint.position.y - lastPoint.position.y, currentPoint.position.x - lastPoint.position.x);

            double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);

//            double angularDifference = currentPoint.position.angle - lastPoint.position.angle;
//            double angularVelocity = calculateNewVelocity(angularDifference, angularAcceleration, lastPoint.velocity.angle);
            // TODO: handle angular velocity after x,y

            Pos2D velocity = new Pos2D(xVelocity,yVelocity,0);
            currentPoint.velocity = velocity;
        }
        return path;
    }


    private double calculateNewVelocity(double dTraveled, double acceleration, double lastVelocity){
        return Math.sqrt(Math.pow(lastVelocity, 2) + 2 * (acceleration * dTraveled));
    }


}
