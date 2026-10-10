package com.broombots.sweep.Classes;

// Represents a position or vector 2d, any unit.
public class Pos2D {
    public double x, y, angle;
    public Pos2D(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
    public static double getDistanceBetweenCoordinates(Pos2D c1, Pos2D c2){
        return Math.hypot(c2.x - c1.x, c2.y - c1.y);
    }
    public double getMagnitude(){
        return Math.hypot(x,y);
    }
    public Pos2D rotateAroundZeroBy(double degrees){
        double translationAngle = Math.toRadians(degrees);
        double x = this.x * Math.cos(translationAngle) - this.y * Math.sin(translationAngle);
        double y = this.x * Math.sin(translationAngle) + this.y * Math.cos(translationAngle);
        return new Pos2D(x, y, this.angle);

    }
    public static Pos2D lerpPos2D(double ratio, Pos2D p1, Pos2D p2){
        return new Pos2D(
                lerp(p1.x,p2.x,ratio),
                lerp(p1.y,p2.y,ratio),
                lerp(p1.angle,p2.angle,ratio)
        );
    }
    public static double getMovementDirectionRad(Pos2D p1, Pos2D p2){
        return Math.atan2(p1.y - p2.y, p1.x - p2.x);
    }
    private static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (end-start) * xInRange;
    }

}
