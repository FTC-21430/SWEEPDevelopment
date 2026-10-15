package com.broombots.sweep.Movement;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.SWEEPTimer;

public class SequenceInterpreter {
    private Sequence currentSequence;
    private SWEEPTimer sweepTimer;
    private RobotMovementController controller;
    public SequenceInterpreter(RobotMovementParameters robotMovementParameters){
        currentSequence = null;
        sweepTimer = new SWEEPTimer();
        controller = new RobotMovementController(robotMovementParameters.getPIDCoefficients());
    }
    public void startPath(Sequence sequence){
        if (sequence == null) throw new IllegalArgumentException("Sequence Cannot be Null");
        currentSequence = sequence;
    }
    public Pos2D update(LocalizationPacket localizationPacket){
        if (sweepTimer.getSeconds() > currentSequence.getLastTime()) currentSequence = null;
        if (currentSequence == null) return new Pos2D(0,0,0);

        currentSequence.updateActions(localizationPacket);
        return controller.getRobotDrivePowers(localizationPacket, currentSequence.getMovement(sweepTimer.getSeconds()));
    }

    public String updateSimulation(LocalizationPacket localizationPacket){
        return currentSequence.updateActionsInSimulation(localizationPacket);
    }
    public Pos2D getRobotTargetPosition(){
        return getPathPoint().position;
    }
    public Pos2D getRobotTargetVelocity(){
        return currentSequence.getMovement(sweepTimer.getSeconds()).velocity;
    }
    public PathPoint getPathPoint(){
        return currentSequence.getMovement(sweepTimer.getSeconds());
    }
    public boolean hasActiveSequence(){
        return currentSequence != null;
    }
    public void addToCurrentSequence(Sequence sequence){
        //TODO merge sequences together in real time
    }
}
