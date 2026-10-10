package com.broombots.sweep.Movement;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.Timer;

public class SequenceInterpreter {
    private Sequence currentSequence;
    private Timer timer;
    private RobotMovementController controller;
    public SequenceInterpreter(RobotMovementParameters robotMovementParameters){
        currentSequence = null;
        timer = new Timer();
        controller = new RobotMovementController(robotMovementParameters.getPIDCoefficients());
    }
    public void startPath(Sequence sequence){
        if (sequence == null) throw new IllegalArgumentException("Sequence Cannot be Null");
        currentSequence = sequence;
    }
    public Pos2D update(LocalizationPacket localizationPacket){
        if (currentSequence == null) return new Pos2D(0,0,0);
        currentSequence.updateActions(localizationPacket);
        return controller.getRobotDrivePowers(localizationPacket, currentSequence.getMovement(timer.getSeconds()));
    }

    public String updateSimulation(LocalizationPacket localizationPacket){
        return currentSequence.updateActionsInSimulation(localizationPacket);
    }
    public Pos2D getRobotTargetPosition(){
        return getPathPoint().position;
    }
    public Pos2D getRobotTargetVelocity(){
        return currentSequence.getMovement(timer.getSeconds()).velocity;
    }
    public PathPoint getPathPoint(){
        return currentSequence.getMovement(timer.getSeconds());
    }
}
