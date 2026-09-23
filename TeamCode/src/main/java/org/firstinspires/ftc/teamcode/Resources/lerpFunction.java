package org.firstinspires.ftc.teamcode.Resources;

import java.util.ArrayList;

public class lerpFunction {
    private final ArrayList<Double> keyedPoints;
    private final ArrayList<Double> resultValues;

    public lerpFunction(ArrayList keyedPoints, ArrayList resultValues){
        if (keyedPoints.size() != resultValues.size()) throw new IllegalArgumentException("both keyed points and keyed values lists must be the same size");
        this.keyedPoints = keyedPoints;
        this.resultValues = resultValues;
    }


    public double getValue(double key){
        if (keyedPoints.contains(key)) return resultValues.get(resultValues.indexOf(key));

        // Binary search
        int low = 0; // first known key index
        int high = keyedPoints.size(); // last known key index
        while (high-low > 1){
            int mid = (int)(high-low)/2;
            if (key > keyedPoints.get(mid)){
                low = mid;
            }else{
                high = mid;
            }
        }

        double lowKey = keyedPoints.get(low);
        double highKey = keyedPoints.get(high);
        double ratio = (key - lowKey) / (highKey - lowKey);
        return lerp(resultValues.get(low), resultValues.get(high), ratio);
    }
    private static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x);
        return start + (end - start) * xInRange;
    }
}
