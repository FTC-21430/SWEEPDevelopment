package com.broombots.sweep.Builder;

import java.util.ArrayList;

public class TimeProfile {
    ArrayList<MovementPoint> movementPoints = new ArrayList<>();
    ArrayList<Double> timeKey = new ArrayList<>();

    public void addPoint(MovementPoint point, double time){
        movementPoints.add(point);
        timeKey.add(time);
    }
    public MovementPoint getPointAtTime(double time){
        int low = 0;
        int high = timeKey.size()-1;
        while (high - low > 1){

            int mid = low + (high-low)/2;
            if (time > timeKey.get(mid)){
                low = mid;
            }else{
                high = mid;
            }
        }
        double first = timeKey.get(low);
        double second = timeKey.get(high);
        double ratio = second!=first?(time-first)/(second-first):0.0;


        return MovementPoint.lerpMovementPoint(movementPoints.get(low),movementPoints.get(high),ratio);
    }
    public MovementMap normalizeTimeProfile(double sampleRate){
        if (sampleRate <= 1e-9) throw new IllegalArgumentException("sampleRate cannot be less than one EPSILON");
        MovementMap result = new MovementMap(sampleRate);
        System.out.println("Time Profile MaxVelY: " + movementPoints.get(movementPoints.size()-1).getVelY());
        for (double t = 0.0; t < getMaxTime()+1e-9; t += sampleRate){
            result.addMovementPoint(getPointAtTime(t));
        }
        return result;
    }
    public double getMaxTime(){
        return timeKey.get(timeKey.size()-1);
    }
}
