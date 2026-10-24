package com.broombots.sweep.Movement;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.SWEEPTimer;

import java.nio.file.Path;

public class SequenceInterpreter {
    private Sequence currentSequence;
    private SWEEPTimer sweepTimer;
    private RobotMovementController controller;
    private RobotMovementParameters robotMovementParameters;
    public SequenceInterpreter(RobotMovementParameters robotMovementParameters){
        this.robotMovementParameters = robotMovementParameters;
        currentSequence = null;
        sweepTimer = new SWEEPTimer();
        controller = new RobotMovementController(robotMovementParameters.getPIDCoefficients());
    }
    public void startPath(Sequence sequence){
        if (sequence == null) throw new IllegalArgumentException("Sequence Cannot be Null");
        currentSequence = sequence;
        sweepTimer.reset();
    }
    public Pos2D update(LocalizationPacket localizationPacket){

        controller.updateMovementCoeffs(robotMovementParameters.getPIDCoefficients());
        if (currentSequence == null) return new Pos2D(0,0,0);
        currentSequence.updateActions(localizationPacket, sweepTimer.getSeconds());
        Pos2D driveTrainMovement = controller.getRobotDrivePowers(localizationPacket, currentSequence.getMovement(sweepTimer.getSeconds()));
        if (sweepTimer.getSeconds() > currentSequence.getLastTime()) currentSequence = null;
        return driveTrainMovement;
    }

    public String updateSimulation(LocalizationPacket localizationPacket){
        return currentSequence.updateActionsInSimulation(localizationPacket, sweepTimer.getSeconds());
    }
    public Pos2D getRobotTargetPosition(){
        return getPathPoint().position;
    }
    public Pos2D getRobotTargetVelocity(){
        return currentSequence.getMovement(sweepTimer.getSeconds()).velocity;
    }
    public PathPoint getPathPoint(){
        if (currentSequence == null){
            PathPoint zero = new PathPoint();
            zero.position = new Pos2D(0,0,0);
            zero.velocity = new Pos2D(0,0,0);
            zero.time = 0;
            return zero;
        }
        return currentSequence.getMovement(sweepTimer.getSeconds());
    }
    public boolean hasActiveSequence(){
        return currentSequence != null;
    }
    /**
     * Appends more movement to the sequence that is currently running.
     * The new path starts from the last point of the running sequence, so the combined path stays continuous.
     * @param sequenceBuilder the builder describing the movement to append; its own start is ignored
     * @throws IllegalStateException if no sequence is running, use startPath() to begin one instead
     */
    public void addToCurrentSequence(SequenceBuilder sequenceBuilder){
        if (currentSequence == null) throw new IllegalStateException("No active sequence to add to, use startPath() first");
        currentSequence.addToSequence(sequenceBuilder);
    }
}
