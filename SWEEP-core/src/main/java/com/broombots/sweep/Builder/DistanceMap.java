package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Splines.Segment;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

public class DistanceMap {
    // scope not defined because they need to be accessed by a class of this type.
    ArrayList<Pos2D> positions =new ArrayList<>();
    ArrayList<Double> distances = new ArrayList<>();

    private final double tSampleRate = 0.01;
    public DistanceMap(Segment segment){
        addSegmentToMap(segment);
    }
    public double getMaxDistance(){ // return end of distance array
        return distances.get(distances.size()-1);
    }
    public double getMinDistance(){
        return distances.get(0);
    }
    public Pos2D getPositionAtDistance(double distance){
        if (isDistanceCalculated(distance)) return positions.get(distances.indexOf(distance));
        Double[] closestDistances = closestDistancesTo(distance);
        double ratio = getPartialRatio(distance, closestDistances);
        return Pos2D.lerpPos2D(ratio, positions.get(distances.indexOf(closestDistances[0])), positions.get(distances.indexOf(closestDistances[1])));
    }
    private double getPartialRatio(double distance, Double[] closestDistances){
        return (distance-closestDistances[0])/(closestDistances[1]-closestDistances[0]);
    }
    private void addSegmentToMap(Segment segment){
        double currentDistance = 0;
        for (double t = 0; t <= 1; t += tSampleRate){
            positions.add(segment.getPosition(t));
            currentDistance += segment.calculateDistance(t-tSampleRate,t);
            distances.add(currentDistance);
//            curvatures.add(getCurvature(segment, t)); // left out at the current stage of development
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

    private double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (end-start) * xInRange;
    }
    private Pos2D lerpPos2D(Pos2D start, Pos2D end, double x){
        return new Pos2D(
                lerp(start.x,end.x,x),
                lerp(start.y,end.y,x),
                lerp(start.angle, end.angle, x)
        );
    }


    // not used currently
    ArrayList<Double> curvatures = new ArrayList<>();
    private Double[] closestCurvaturesTo(double distance){
        Double[] bracketDistances = closestDistancesTo(distance);
        Double[] result = new Double[bracketDistances.length];
        for (int i = 0; i < bracketDistances.length; i++){
            result[i] = curvatures.get(distances.indexOf(bracketDistances[i]));
        }
        return result;
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
        return distanceAtLocalMaximas;
    }
    public double getCurvatureAtDistance(double distance){
        if (isDistanceCalculated(distance)) return curvatures.get(distances.indexOf(distance));

        Double[] closestCurvatures = closestCurvaturesTo(distance);
        double ratio = getPartialRatio(distance, closestCurvatures);
        return lerp(closestCurvatures[0], closestCurvatures[1], ratio);
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
