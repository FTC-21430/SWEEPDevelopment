package com.broombots.sweep.Classes;

import java.util.ArrayList;

public class LerpFunctionBuilder {
    ArrayList<Double> keyedPoints = new ArrayList<>();
    ArrayList<Double> resultPoints = new ArrayList<>();
    // Add it in order please.
    public LerpFunctionBuilder addPoint(double key, double result){
        keyedPoints.add(key);
        resultPoints.add(result);
        return this;
    }
    public LerpFunction build(){
        return new LerpFunction(keyedPoints, resultPoints);
    }

}
