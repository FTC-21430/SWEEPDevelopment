package com.broombots.sweep.Classes;

/** Immutable sample of robot pose and field-frame translational velocity. */
@SuppressWarnings("unused")
public class LocalizationPacket {
    /** Field x and y position in inches, heading yaw in degrees, and field-frame velocities in inches per second. */
    private final double x,y,yaw,velX,velY;
    /** Creates a localization sample using SWEEP's field units.
     * @param x field X position in inches
     * @param y field Y position in inches
     * @param yaw robot heading in degrees
     * @param velocityX velocity along the field X axis in inches per second
     * @param velocityY velocity along the field Y axis in inches per second
     */
    public LocalizationPacket(double x, double y, double yaw, double velocityX, double velocityY){
       this.x = x;
       this.y = y;
       this.yaw = yaw;
       this.velX = velocityX;
       this.velY = velocityY;

    }
    /** Returns the field X position.
     * @return X position in inches */
    public double getX() {
        return x;
    }
    /** Returns the field Y position.
     * @return Y position in inches */
    public double getY() {
        return y;
    }
    /** Returns the robot heading.
     * @return yaw in degrees */
    public double getYaw() {
        return yaw;
    }
    /** Returns the field-frame velocity along X.
     * @return X velocity in inches per second */
    public double getVelX() {
        return velX;
    }
    /** Returns the field-frame velocity along Y.
     * @return Y velocity in inches per second */
    public double getVelY() {
        return velY;
    }
    /** Converts the pose components to a {@link Pos2D}.
     * @return position and heading, in inches and degrees */
    public Pos2D getCoordinate(){
        return new Pos2D(x,y,yaw);
    }
}
