package org.firstinspires.ftc.teamcode.Resources;

import java.util.ArrayList;

public class lerpFunctionBuilder {
    ArrayList<Double> keyedPoints = new ArrayList<>();
    ArrayList<Double> resultPoints = new ArrayList<>();
    // Add it in order please.
    public lerpFunctionBuilder addPoint(double key, double result){
        keyedPoints.add(key);
        resultPoints.add(result);
        return this;
    }
    public lerpFunction build(){
        return new lerpFunction(keyedPoints, resultPoints);
    }

}
