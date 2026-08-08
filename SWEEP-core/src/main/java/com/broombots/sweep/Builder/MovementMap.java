package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;

import java.util.ArrayList;

public class MovementMap {

    private static final double EPSILON = 1e-9;
    // The time between each movementPoint. This time must be consistent between all points
    private final double sampleRate;
    private final ArrayList<MovementPoint> movementMap = new ArrayList<>();

    public MovementMap(double sampleRate){
        this.sampleRate = sampleRate;
    }
    public void addMovementPoint(MovementPoint movementPoint){
        movementMap.add(movementPoint);
    }
    public void addMovementPoints(ArrayList<MovementPoint> points){
        movementMap.addAll(points);
    }
    public MovementPoint getPoint(double unit){
        // round unit into the nearest integer value that aligns with the index of the last exact movement point
        int idxFloor = (int)(unit/sampleRate);
        // check if there is another point after this unit, which there should be
        if (idxFloor < movementMap.size()-1){
            MovementPoint lastPoint = movementMap.get(idxFloor);
            MovementPoint nextPoint = movementMap.get(idxFloor+1);
            return lerpMovementPoint(lastPoint,nextPoint, (unit/sampleRate) - (double)idxFloor);
        }
        else {
            return movementMap.get(movementMap.size()-1);
        }
    }
    public double getSampleRate(){
        return sampleRate;
    }
    public static MovementMap combine(MovementMap first, MovementMap second){
        if (Math.abs(first.getSampleRate()- second.getSampleRate())>EPSILON) throw new RuntimeException("MovementMap sample rates do not match");
        MovementMap result = new MovementMap(first.getSampleRate());
        result.addMovementPoints(first.getAllPoints());
        result.addMovementPoints(second.getAllPoints());
        return result;
    }
    public static MovementMap getMinimum(MovementMap[] maps){
        if (maps == null) throw new RuntimeException("ERROR, passed maps was NULL");
        if (maps.length == 0) throw new RuntimeException("Cannot compare no maps");
        if (maps.length == 1) return maps[0];
        //Ensure all maps are non-null
        for (MovementMap map : maps){
            if (map == null) throw new IllegalArgumentException("OUCH, one of the maps was NULL");
        }
        // Ensure that of the maps share the same sample rate.
        double firstRate = maps[0].getSampleRate();
        // Ensure that all the maps are of the same size
        int firstSize = maps[0].getAllPoints().size();
        for (MovementMap map : maps){
            if (Math.abs(map.getSampleRate()-firstRate)>EPSILON) throw new RuntimeException("Sample rates do not match between the different movement maps");
            if (map.getAllPoints().size() != firstSize) throw new RuntimeException("Map sizes do not match, ensure that there are the same number of elements in each map");

        }

        MovementMap results = new MovementMap(firstRate);

        for (int i = 0; i < maps[0].getAllPoints().size(); i++){
            int bestIdx = 0;
            double best = maps[0].getAllPoints().get(i).getVelocityMagnitude();
            for (int j = 1; j < maps.length; j++){
                if (maps[j].getAllPoints().get(i).getVelocityMagnitude() < best){
                    best = maps[j].getAllPoints().get(i).getVelocityMagnitude();
                    bestIdx = j;
                }
            }
            results.addMovementPoint(maps[bestIdx].getAllPoints().get(i));
        }
        return results;
    }
    public ArrayList<MovementPoint> getAllPoints(){
        return new ArrayList<>(movementMap);
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
        return start + (end-start) * xInRange;
    }

}
