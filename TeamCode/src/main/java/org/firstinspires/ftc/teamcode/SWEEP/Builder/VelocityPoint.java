package org.firstinspires.ftc.teamcode.SWEEP.Builder;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;

public class VelocityPoint {
    private final Coordinate position;
    private final double velX, velY, velAngle;
    public VelocityPoint(Coordinate position, double velX, double velY, double velAngle){
        this.position = position;
        this.velX = velX;
        this.velY = velY;
        this.velAngle = velAngle;
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
}
