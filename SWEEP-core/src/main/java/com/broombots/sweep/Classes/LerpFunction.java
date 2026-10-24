package com.broombots.sweep.Classes;

import java.util.ArrayList;

/** Interpolates result values linearly between ordered scalar keys. */
public class LerpFunction {
    /** Keys defining interpolation points, in ascending order. */
    private final ArrayList<Double> keyedPoints;
    /** Results corresponding by index to {@link #keyedPoints}. */
    private final ArrayList<Double> resultValues;

    /** Creates an interpolator from equally sized, ascending key and result lists.
     * @param keyedPoints interpolation keys in ascending order
     * @param resultValues results corresponding to each key
     * @throws IllegalArgumentException if the lists have different sizes
     */
    public LerpFunction(ArrayList<Double> keyedPoints, ArrayList<Double> resultValues){
        if (keyedPoints.size() != resultValues.size()) throw new IllegalArgumentException("both keyed points and keyed values lists must be the same size");
        this.keyedPoints = keyedPoints;
        this.resultValues = resultValues;
    }

    /** Gets the interpolated result for a key, clamping outside keys to endpoint values.
     * @param key query key in the same units as the configured keys
     * @return interpolated result in the same units as the configured results
     * @throws IllegalStateException if no points were configured
     */
    public double getValue(double key){
        if (keyedPoints.isEmpty()) throw new IllegalStateException("LerpFunction has no points");
        if (keyedPoints.size() == 1) return resultValues.get(0);

        if (key <= keyedPoints.get(0)) return resultValues.get(0);
        int lastIndex = keyedPoints.size() - 1;
        if (key >= keyedPoints.get(lastIndex)) return resultValues.get(lastIndex);

        int low = 0;
        int high = lastIndex;
        while (high - low > 1){
            int mid = low + (high - low) / 2;
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
    /** Interpolates between two scalar values using the supplied ratio. */
    private static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x);
        return start + (end - start) * xInRange;
    }
}
