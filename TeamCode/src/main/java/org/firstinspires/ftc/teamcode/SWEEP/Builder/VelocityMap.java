package org.firstinspires.ftc.teamcode.SWEEP.Builder;

import java.util.Dictionary;

public class VelocityMap {
    private Dictionary<Double, VelocityPoint> velocityMap;
    public VelocityMap(){

    }
    public void addVelocity(double time, VelocityPoint velocityPoint){
        velocityMap.put(time,velocityPoint);
    }
    public void getVelocityAt(double time){

    }
    public void getPositionAt(double time){

    }
}
