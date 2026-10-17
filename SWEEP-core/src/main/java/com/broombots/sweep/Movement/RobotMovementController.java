package com.broombots.sweep.Movement;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPTimer;

import org.ejml.simple.SimpleMatrix;

public class RobotMovementController {
    SimpleMatrix movementCoeffs;
    SWEEPPIDController positionXController;

    SWEEPPIDController positionYController;
    SWEEPPIDController positionAngleController;
    SWEEPPIDController velocityXController;
    SWEEPPIDController velocityYController;
    public RobotMovementController(SimpleMatrix movementCoeffs){
        this.movementCoeffs = movementCoeffs;
        /** coefficient matrix in form
         * xy : p i d
         * angle : p i d
         * vX X : p i d

         */

        positionXController = new SWEEPPIDController(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2), new SWEEPTimer(),false);
        positionYController = new SWEEPPIDController(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2), new SWEEPTimer(),false);
        positionAngleController = new SWEEPPIDController(movementCoeffs.get(1,0), movementCoeffs.get(1,1), movementCoeffs.get(1,2), new SWEEPTimer(),true);
        velocityXController = new SWEEPPIDController(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2), new SWEEPTimer(),false);
        velocityYController = new SWEEPPIDController(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2), new SWEEPTimer(),false);
    }
    public void updateMovementCoeffs(SimpleMatrix movementCoeffs){
        this.movementCoeffs = movementCoeffs;
        positionXController.updatePIDConstants(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2));
        positionYController.updatePIDConstants(movementCoeffs.get(0,0), movementCoeffs.get(0,1), movementCoeffs.get(0,2));
        positionAngleController.updatePIDConstants(movementCoeffs.get(1,0), movementCoeffs.get(1,1), movementCoeffs.get(1,2));
        velocityXController.updatePIDConstants(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2));
        velocityYController.updatePIDConstants(movementCoeffs.get(2,0), movementCoeffs.get(2,1), movementCoeffs.get(2,2));

    }
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

        // When robot has drive train forward (+ values moves forward) and sideways( where + goes to the right of the robot)
        Pos2D fieldCentric = new Pos2D(-sidePower,forwardPower,anglePower);
        return fieldCentric.rotateAroundZeroBy(-localizationPacket.getYaw());
    }
}
