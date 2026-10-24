package com.broombots.sweep.Classes;

import java.util.ArrayList;

/** Builds a {@link LerpFunction} by collecting key/result pairs in ascending key order. */
public class LerpFunctionBuilder {
    /** Interpolation keys, which must be added in ascending order. */
    private ArrayList<Double> keyedPoints = new ArrayList<>();
    /** Results corresponding by index to {@link #keyedPoints}. */
    private ArrayList<Double> resultPoints = new ArrayList<>();

    /** Adds one interpolation point; keys must be supplied in ascending order.
     * @param key interpolation key
     * @param result result value associated with the key
     * @return this builder for chaining
     */
    public LerpFunctionBuilder addPoint(double key, double result){
        keyedPoints.add(key);
        resultPoints.add(result);
        return this;
    }
    /** Builds the interpolator from all points added so far.
     * @return a function that interpolates the configured points */
    public LerpFunction build(){
        return new LerpFunction(keyedPoints, resultPoints);
    }

}
