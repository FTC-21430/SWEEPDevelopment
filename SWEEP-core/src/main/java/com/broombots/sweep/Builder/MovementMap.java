package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;

import java.util.ArrayList;

public class MovementMap {
    // The time between each movementPoint. This time must be consistent between all points
    private final double sampleRate;
    private ArrayList<MovementPoint> movementMap;

    public MovementMap(double sampleRate){
        this.sampleRate = sampleRate;
    }
    public void addMovementPoint(MovementPoint movementPoint){
        movementMap.add(movementPoint);
    }
    public void addMovementPoints(ArrayList<MovementPoint> points){
        movementMap.addAll(points);
    }
    public MovementPoint getPoint(double time){
        // round time into the nearest integer value that aligns with the index of the last exact movement point
        int idxFloor = (int)(time/sampleRate);
        // check if there is another point after this time, which there should be
        if (idxFloor < movementMap.size()-1){
            MovementPoint lastPoint = movementMap.get(idxFloor);
            MovementPoint nextPoint = movementMap.get(idxFloor+1);
            return lerpMovementPoint(lastPoint,nextPoint, (time/sampleRate) - (double)idxFloor);
        }
        else {
            return movementMap.get(movementMap.size()-1);
        }
    }
    public double getSampleRate(){
        return sampleRate;
    }
    public static MovementMap combine(MovementMap first, MovementMap second){
        if (first.sampleRate != second.sampleRate) throw new RuntimeException("MovementMap sample rates do not match");
        MovementMap result = new MovementMap(first.getSampleRate());
        result.addMovementPoints(first.getAllPoints());
        result.addMovementPoints(second.getAllPoints());
        return result;
    }
    public ArrayList<MovementPoint> getAllPoints(){
        return movementMap;
    }
    private MovementPoint lerpMovementPoint(MovementPoint startPoint, MovementPoint endPoint, double x){
        double posX = lerp(startPoint.getPosition().getX(), endPoint.getPosition().getX(), x);
        double posY = lerp(startPoint.getPosition().getY(), endPoint.getPosition().getY(), x);
        double posAngle = lerp(startPoint.getPosition().getAngle(), endPoint.getPosition().getAngle(),x);
        double velX = lerp(startPoint.getVelX(), endPoint.getVelX(), x);
        double velY = lerp(startPoint.getVelY(), endPoint.getVelY(), x);
        double velAngle = lerp(startPoint.getVelAngle(), endPoint.getVelAngle(),x);
        double accelX = lerp(startPoint.getAccelX(), endPoint.getAccelX(), x);
        double accelY = lerp(startPoint.getAccelY(),endPoint.getAccelY(), x);
        double accelAngle = lerp(startPoint.getAccelAngle(), endPoint.getAccelAngle(), x);

        return new MovementPoint(new Coordinate(posX,posY, posAngle), velX,velY,velAngle,accelX,accelY,accelAngle);
    }
    private double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (start-end) * xInRange;
    }

}
