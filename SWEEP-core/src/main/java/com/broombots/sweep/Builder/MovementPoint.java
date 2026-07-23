package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;

public class MovementPoint {
    private final Coordinate position;
    private final double velX, velY, velAngle;
    private final double accelX, accelY, accelAngle;
    public MovementPoint(Coordinate position, double velX, double velY, double velAngle, double accelX, double accelY, double accelAngle){
        this.position = position;
        this.velX = velX;
        this.velY = velY;
        this.velAngle = velAngle;
        this.accelX = accelX;
        this.accelY = accelY;
        this.accelAngle = accelAngle;

    }
    public Coordinate getPosition(){
        return position;
    }
    public double getVelX(){
        return velX;
    }
    public double getVelY(){
        return velY;
    }
    public double getVelAngle(){
        return velAngle;
    }

    public double getAccelX() {
        return accelX;
    }

    public double getAccelY() {
        return accelY;
    }

    public double getAccelAngle() {
        return accelAngle;
    }
}
