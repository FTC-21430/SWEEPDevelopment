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
    private final double tSampleRate = 0.01;
    public DistanceMap(Segment[] segments){
        double segmentDistance = 0;
        for (Segment seg : segments){
            addSegmentToMap(seg, segmentDistance);
            segmentDistance = getMaxDistance();
        }
    }
    public DistanceMap(Segment segment){
        addSegmentToMap(segment, 0);
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
    public ArrayList<Double> getSegmentDistancesWithLocalMaximaCurvature(){
        ArrayList<Double> localMaximaCurvatures = new ArrayList<>(); // makes shallow list copy that can be sorted because double is an immutable type
        ArrayList<Double> distanceAtLocalMaximas = new ArrayList<>();
        for (int i = 1; i < curvatures.size()-2; i++){
            if (Math.abs(curvatures.get(i)) > Math.abs(curvatures.get(i-1)) && Math.abs(curvatures.get(i)) > Math.abs(curvatures.get(i+1))){
                localMaximaCurvatures.add(Math.abs(curvatures.get(i)));
                distanceAtLocalMaximas.add(distances.get(i));
            }
        }
        System.out.println(localMaximaCurvatures.size());
//        ArrayList<Double> result = new ArrayList<>();
//        for (int j = 0; j < Math.min(2, localMaximaCurvatures.size()); j++){
//            int bestIdx = 0;
//            for (int i = 1; i < localMaximaCurvatures.size(); i++){
//                if (localMaximaCurvatures.get(i) > localMaximaCurvatures.get(bestIdx)){
//                    bestIdx = i;
//                }
//            }
//            result.add(distanceAtLocalMaximas.get(bestIdx));
//            localMaximaCurvatures.remove(bestIdx);
//            distanceAtLocalMaximas.remove(bestIdx);
//        }
//        while (result.size() < 2){
//            result.add(0.0);
//        }
        return distanceAtLocalMaximas;
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
        ArrayList<Double> search = new ArrayList<>();
        int low = 0;
        int high = distances.size() - 1;

        while (high - low > 1){
            int mid = (low+high) / 2;
            if (distance >= distances.get(mid)){
                low = mid;
            } else {
                high = mid;
            }
        }
        search.add(distances.get(low));
        search.add(distances.get(high));
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
    //Curvature is 1/R with R being the radius of the circle that will best fit the curve. expect values to be greater for tighter curves, and close to zero for straight lines. Range should be about -2, 2 for normal curves on the field,
    // Play around with curvature and cubic splines in this colab document https://colab.research.google.com/drive/1WzmwwOckUa04UeXFfJ6J6wBxXhSQYZvf?usp=sharing
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
        if (denominator < 10e-9) return 0.0;
        return Math.abs(x1 * y2 - y1 * x2) / denominator;
    }
}
