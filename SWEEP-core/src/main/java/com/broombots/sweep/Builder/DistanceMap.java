package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

public class DistanceMap {
    ArrayList<Coordinate> coordinates =new ArrayList<>();
    ArrayList<Double> distances = new ArrayList<>();
    ArrayList<Double> curvatures = new ArrayList<>();

    ArrayList<Double> segmentSplitDistances = new ArrayList<>();
    private final double tSampleRate = 0.01;
    private final double maxDistance;
    public DistanceMap(Segment[] segments){
        double segmentDistance = 0;
        for (Segment seg : segments){
            addSegmentToMap(seg, segmentDistance);
            segmentDistance = getMaxDistance();
            segmentSplitDistances.add(segmentDistance);
        }
        maxDistance = segmentDistance;
    }
    public double getMaxDistance(){ // return end of distance array
        return maxDistance;
    }
    public Coordinate getPositionAtDistance(double distance){
        if (isDistanceCalculated(distance)) return coordinates.get(distances.indexOf(distance));
        Double[] closestDistances = closestDistancesTo(distance);
        double ratio = getPartialRatio(distance);
        return lerpCoordinate(coordinates.get(distances.indexOf(closestDistances[0])),coordinates.get(distances.indexOf(closestDistances[1])), ratio);
    }
    public double getCurvatureAtDistance(double distance){
        if (isDistanceCalculated(distance)) return curvatures.get(distances.indexOf(distance));

        Double[] closestCurvatures = closestCurvaturesTo(distance);
        double ratio = getPartialRatio(distance);
        return lerp(closestCurvatures[0], closestCurvatures[1], ratio);
    }
    private double getPartialRatio(double distance){
        Double[] closestDistances = closestDistancesTo(distance);
        return (distance-closestDistances[0])/(closestDistances[1]-closestDistances[0]);
    }
    public double[] getDistanceBoundsForSegmentIndex(int idx){
        if (idx < 0 || idx > segmentSplitDistances.size()-1) throw new RuntimeException("Provided Index is out bounds");
        if (idx == 0){ // At the start of the path where idx -1 is out of bounds, but is going to evaluate to 0
            return new double[]{
                    0,
                    segmentSplitDistances.get(0)
            };
        }else {
            return new double[]{
                    segmentSplitDistances.get(idx - 1),
                    segmentSplitDistances.get(idx)
            };
        }
    }
    private void addSegmentToMap(Segment segment, double startDistance){
        for (double t = 0; t <= 1; t += tSampleRate){
            coordinates.add(segment.getPosition(t));
            distances.add(segment.calculateDistance(0,t) + startDistance);
            curvatures.add(getCurvature(segment, t));
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
    private Double[] closestCurvaturesTo(double distance){
        if (isDistanceCalculated(distance)) return new Double[]{distance};
        ArrayList<Double> search = curvatures; // TODO: check for pointer in the copy, we need a copy, not a pointer.
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

    private double getCurvature(Segment segment, double t){
        /**
         * ax, bx, cx, dx,
         * ay, by, cy, dy
         */
        SimpleMatrix f = segment.getSplineFormula(); // Formula
        double x1 = 3.0 * f.get(0,0) * t * t + 2.0 * f.get(0,1) * t + f.get(0,2); // first derivative of x
        double x2 = 6.0 * f.get(0,0) * t + 2.0 * f.get(0,1); // second derivative of x
        double y1 = 3.0 * f.get(1,0) * t * t + 2.0 * f.get(1,1) * t + f.get(1,2); // first derivative of x
        double y2 = 6.0 * f.get(1,0) * t + 2.0 * f.get(1,1); // second derivative of x

        double denominator = Math.pow(x1 * x1 + y1 * y1,1.5);
        if (denominator == 0.0) return 0.0;
        return Math.abs(x1 * y2 - y1 * x2) / denominator;
    }
}
