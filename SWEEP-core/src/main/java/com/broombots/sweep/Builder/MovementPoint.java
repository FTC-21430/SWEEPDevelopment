package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Pos2D;

public class MovementPoint {
    private final Pos2D position;
    private final double velX, velY, velAngle;
    private double accelX, accelY, accelAngle;
    public MovementPoint(Pos2D position, double velX, double velY, double velAngle, double accelX, double accelY, double accelAngle){
        this.position = position;
        this.velX = velX;
        this.velY = velY;
        this.velAngle = velAngle;
        this.accelX = accelX;
        this.accelY = accelY;
        this.accelAngle = accelAngle;

    }
    public Pos2D getPosition(){
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
    public void setAcceleration(double x, double y, double angle){
        accelX = x;
        accelY = y;
        accelAngle = angle;
    }
    public double getVelocityMagnitude(){
        return Math.hypot(velX,velY);
    }
    public static MovementPoint lerpMovementPoint(MovementPoint startPoint, MovementPoint endPoint, double x){
        double posX = lerp(startPoint.getPosition().x, endPoint.getPosition().x, x);
        double posY = lerp(startPoint.getPosition().y, endPoint.getPosition().y, x);
        double posAngle = lerp(startPoint.getPosition().angle, endPoint.getPosition().angle,x);
        double velX = lerp(startPoint.getVelX(), endPoint.getVelX(), x);
        double velY = lerp(startPoint.getVelY(), endPoint.getVelY(), x);
        double velAngle = lerp(startPoint.getVelAngle(), endPoint.getVelAngle(),x);
        double accelX = lerp(startPoint.getAccelX(), endPoint.getAccelX(), x);
        double accelY = lerp(startPoint.getAccelY(),endPoint.getAccelY(), x);
        double accelAngle = lerp(startPoint.getAccelAngle(), endPoint.getAccelAngle(), x);

        return new MovementPoint(new Pos2D(posX,posY, posAngle), velX,velY,velAngle,accelX,accelY,accelAngle);
    }
    public static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (end-start) * xInRange;
    }
}
