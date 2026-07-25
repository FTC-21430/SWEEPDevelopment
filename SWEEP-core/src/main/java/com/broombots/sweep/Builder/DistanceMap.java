package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Splines.Segment;

import java.util.ArrayList;

public class DistanceMap {
    ArrayList<Coordinate> coordinates =new ArrayList<>();
    ArrayList<Double> distances = new ArrayList<>();
    private final double tSampleRate = 0.01;
    private final double maxDistance;
    public DistanceMap(Segment[] segments){
        double segmentDistance = 0;
        for (Segment seg : segments){
            addSegmentToMap(seg, segmentDistance);
            segmentDistance = getMaxDistance();
        }
        maxDistance = segmentDistance;
    }
    public double getMaxDistance(){ // return end of distance array
        return maxDistance;
    }
    public Coordinate getPositionAtDistance(double distance){
        if (isDistanceCalculated(distance)) return coordinates.get(distances.indexOf(distance));

        Double[] closestDistances = closestDistancesTo(distance);
        double ratio = (distance-closestDistances[0])/(closestDistances[1]-closestDistances[0]);
        return lerpCoordinate(coordinates.get(distances.indexOf(closestDistances[0])),coordinates.get(distances.indexOf(closestDistances[1])), ratio);
    }
    private void addSegmentToMap(Segment segment, double startDistance){
        for (double t = 0; t <= 1; t += tSampleRate){
            coordinates.add(segment.getPosition(t));
            distances.add(segment.calculateDistance(0,t) + startDistance);
        }
    }
    private boolean isDistanceCalculated(double distance){
        for (double calculatedDistance : distances){
            if (calculatedDistance == distance) return true;
        }
        return false;
    }
    private Double[] closestDistancesTo(double distance){
        if (isDistanceCalculated(distance)) return new Double[]{distance};
        ArrayList<Double> search = distances; // TODO: check for pointer in the copy, we need a copy, not a pointer.
        int elements = search.size();

        while (elements > 2){
            if (distance >= search.get(elements/2)){
                for (int i = 0; i < elements/2; i++){
                    search.remove(0);
                }
            }
            else{
                for (int i = 0; i < (elements/2)-1; i++){
                    search.remove(elements/2);
                }
            }
            elements = search.size();
        }
       return search.toArray(new Double[0]);
    }

    private double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (start-end) * xInRange;
    }
    private Coordinate lerpCoordinate(Coordinate start, Coordinate end, double x){
        return new Coordinate(
                lerp(start.getX(),end.getX(),x),
                lerp(start.getY(),end.getY(),x),
                lerp(start.getAngle(), end.getAngle(), x)
        );
    }
}
