package com.broombots.sweep.Movement;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPTimer;

import org.ejml.simple.SimpleMatrix;

/**
 * Converts path targets and localization into robot-centric drivetrain powers.
 * Five PID controllers track position x/y, heading, and velocity x/y; position and velocity corrections are blended 2:1
 * before the translation correction is rotated from the field frame into the robot frame.
 */
public class RobotMovementController {
    /** PID controller for field x position, in inches. */
    private final SWEEPPIDController positionXController;
    /** PID controller for field y position, in inches. */
    private final SWEEPPIDController positionYController;
    /** PID controller for robot heading, in degrees. */
    private final SWEEPPIDController positionAngleController;
    /** PID controller for field x velocity, in inches per second. */
    private final SWEEPPIDController velocityXController;
    /** PID controller for field y velocity, in inches per second. */
    private final SWEEPPIDController velocityYController;

    /**
     * Creates the five controllers from a 3-by-3 PID coefficient matrix.
     * @param movementCoeffs matrix rows are position PID (x and y), heading PID, and velocity PID (x and y); columns are P, I, and D
     */
    public RobotMovementController(SimpleMatrix movementCoeffs){
        positionXController = new SWEEPPIDController(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2), new SWEEPTimer(),false);
        positionYController = new SWEEPPIDController(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2), new SWEEPTimer(),false);
        positionAngleController = new SWEEPPIDController(movementCoeffs.get(1,0), movementCoeffs.get(1,1), movementCoeffs.get(1,2), new SWEEPTimer(),true);
        velocityXController = new SWEEPPIDController(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2), new SWEEPTimer(),false);
        velocityYController = new SWEEPPIDController(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2), new SWEEPTimer(),false);
    }
    /**
     * Updates PID coefficients for all five controllers.
     * @param movementCoeffs matrix rows are position PID (x and y), heading PID, and velocity PID (x and y); columns are P, I, and D
     */
    public void updateMovementCoeffs(SimpleMatrix movementCoeffs){
        positionXController.updatePIDConstants(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2));
        positionYController.updatePIDConstants(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2));
        positionAngleController.updatePIDConstants(movementCoeffs.get(1,0), movementCoeffs.get(1,1), movementCoeffs.get(1,2));
        velocityXController.updatePIDConstants(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2));
        velocityYController.updatePIDConstants(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2));

    }
    /**
     * Calculates robot-centric drivetrain powers from the current localization and path target.
     * @param localizationPacket robot pose and velocity; pose uses inches and degrees, velocity uses inches per second
     * @param currentTarget desired pose and velocity in field coordinates
     * @return robot-frame drive powers in {@code Pos2D}: x is robot-left (negative is right), y is robot-forward, and angle is rotation power; translation is not clamped again after rotation
     */
    public Pos2D getRobotDrivePowers(LocalizationPacket localizationPacket, PathPoint currentTarget){

        positionXController.setTarget(currentTarget.position.x);
        positionXController.update(localizationPacket.getX());

        positionYController.setTarget(currentTarget.position.y);
        positionYController.update(localizationPacket.getY());

        positionAngleController.setTarget(currentTarget.position.angle);
        positionAngleController.update(localizationPacket.getYaw());

        velocityXController.setTarget(currentTarget.velocity.x);
        velocityXController.update(localizationPacket.getVelX());

        velocityYController.setTarget(currentTarget.velocity.y);
        velocityYController.update(localizationPacket.getVelY());

        double forwardPower = (2*positionXController.getPower() + velocityXController.getPower())/3;
        double sidePower = (2*positionYController.getPower() + velocityYController.getPower())/3;
        double anglePower = -positionAngleController.getPower(); // angle velocity profiling not complete.

        // The assembled translation is field-oriented; rotate it into the robot frame before returning drive powers.
        Pos2D fieldFramePowers = new Pos2D(-sidePower,forwardPower,anglePower);
        return fieldFramePowers.rotateAroundZeroBy(-localizationPacket.getYaw());
    }
}
