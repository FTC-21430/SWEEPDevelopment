package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Comparator;

public class DistanceMap {
    private ArrayList<Coordinate> coordinates =new ArrayList<>();
    private ArrayList<Double> distances = new ArrayList<>();
    private ArrayList<Double> curvatures = new ArrayList<>();

    // TODO - Can be a single value, which represents the path length of this segment
    ArrayList<Double> segmentSplitDistances = new ArrayList<>();
    private final double tSampleRate = 0.01;
    public DistanceMap(Segment[] segments){
        double segmentDistance = 0;
        for (Segment seg : segments){
            addSegmentToMap(seg, segmentDistance);
            segmentDistance = getMaxDistance();
            segmentSplitDistances.add(segmentDistance);
        }
    }
    public DistanceMap(Segment segment){
        addSegmentToMap(segment, 0);
        segmentSplitDistances.add(getMaxDistance());
    }
    public double getMaxDistance(){ // return end of distance array
        return distances.get(distances.size()-1);
    }
    public double getMinDistance(){
        return distances.get(0);
    }
    public Coordinate getPositionAtDistance(double distance){
        if (isDistanceCalculated(distance)) return coordinates.get(distances.indexOf(distance));
        Double[] closestDistances = closestDistancesTo(distance);
        double ratio = getPartialRatio(distance, closestDistances);
        return lerpCoordinate(coordinates.get(distances.indexOf(closestDistances[0])),coordinates.get(distances.indexOf(closestDistances[1])), ratio);
    }
    public double getCurvatureAtDistance(double distance){
        if (isDistanceCalculated(distance)) return curvatures.get(distances.indexOf(distance));

        Double[] closestCurvatures = closestCurvaturesTo(distance);
        double ratio = getPartialRatio(distance, closestCurvatures);
        return lerp(closestCurvatures[0], closestCurvatures[1], ratio);
    }
    private double getPartialRatio(double distance, Double[] closestDistances){
        return (distance-closestDistances[0])/(closestDistances[1]-closestDistances[0]);
    }
    public double[] getSegmentMaxCurvaturePoints(){
        // TODO - Fix to get 2 local maxima points, not 2 largest points
        ArrayList<Double> curvatureArray = new ArrayList<>(curvatures); // makes shallow list copy that can be sorted because double is an immutable type
        curvatureArray.sort(Comparator.naturalOrder());
        return new double[]{
            distances.get(curvatures.indexOf(curvatureArray.get(curvatureArray.size()-2))),
            distances.get(curvatures.indexOf(curvatureArray.get(curvatureArray.size()-1)))
        };
    }
    private void addSegmentToMap(Segment segment, double startDistance){
        double currentDistance = 0;
        for (double t = 0; t <= 1; t += tSampleRate){
            coordinates.add(segment.getPosition(t));
            currentDistance += segment.calculateDistance(t-tSampleRate,t);
            distances.add(currentDistance);
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
        // TODO - Don't remove elements from distances, just add the 2 elements nearest
        ArrayList<Double> search = new ArrayList<>(distances);
        int elements = search.size();

        while (elements > 2){
            int mid = elements / 2;
            if (distance >= search.get(mid)){
                for (int i = 0; i < mid; i++){
                    search.remove(0);
                }
            } else {
                while (search.size() > mid + 1){
                    search.remove(search.size() - 1);
                }
            }
            elements = search.size();
        }
        return search.toArray(new Double[0]);
    }
    private Double[] closestCurvaturesTo(double distance){
        Double[] bracketDistances = closestDistancesTo(distance);
        Double[] result = new Double[bracketDistances.length];
        for (int i = 0; i < bracketDistances.length; i++){
            result[i] = curvatures.get(distances.indexOf(bracketDistances[i]));
        }
        return result;
    }


    private double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (end-start) * xInRange;
    }
    private Coordinate lerpCoordinate(Coordinate start, Coordinate end, double x){
        return new Coordinate(
                lerp(start.getX(),end.getX(),x),
                lerp(start.getY(),end.getY(),x),
                lerp(start.getAngle(), end.getAngle(), x)
        );
    }

    // TODO - Graph this so we know what values to expect
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
