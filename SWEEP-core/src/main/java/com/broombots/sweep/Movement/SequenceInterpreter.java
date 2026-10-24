package com.broombots.sweep.Movement;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.SWEEPTimer;


/**
 * Runs the active {@link Sequence} against localization and returns drivetrain powers.
 * It owns the sequence timer and {@link RobotMovementController}, advancing actions and clearing completed sequences.
 */
public class SequenceInterpreter {
    /** Sequence currently being followed, or {@code null} when idle. */
    private Sequence currentSequence;
    /** Elapsed sequence timer, in seconds. */
    private final SWEEPTimer sweepTimer;
    /** Converts target state and localization into robot-centric drivetrain powers. */
    private final RobotMovementController controller;
    /** Robot motion limits used to construct and tune the movement controller. */
    private final RobotMovementParameters robotMovementParameters;

    /**
     * Creates an interpreter using the supplied robot motion configuration.
     * @param robotMovementParameters robot motion limits and PID coefficients
     */
    public SequenceInterpreter(RobotMovementParameters robotMovementParameters){
        this.robotMovementParameters = robotMovementParameters;
        currentSequence = null;
        sweepTimer = new SWEEPTimer();
        controller = new RobotMovementController(robotMovementParameters.getPIDCoefficients());
    }
    /**
     * Starts a sequence and resets its elapsed timer.
     * @param sequence compiled sequence to execute
     * @throws IllegalArgumentException if sequence is {@code null}
     */
    public void startPath(Sequence sequence){
        if (sequence == null) throw new IllegalArgumentException("Sequence Cannot be Null");
        currentSequence = sequence;
        sweepTimer.reset();
    }
    /**
     * Advances the active sequence and calculates robot-centric drivetrain powers.
     * Output x is robot-left (negative is right), y is robot-forward, and angle is rotation power; all are power values.
     * Returns zero powers when idle and clears the sequence after its final time.
     * @param localizationPacket current robot pose in inches and degrees and velocity in inches per second
     * @return robot-centric drivetrain powers
     */
    public Pos2D update(LocalizationPacket localizationPacket){

        controller.updateMovementCoeffs(robotMovementParameters.getPIDCoefficients());
        if (currentSequence == null) return new Pos2D(0,0,0);
        currentSequence.updateActions(localizationPacket, sweepTimer.getSeconds());
        Pos2D driveTrainMovement = controller.getRobotDrivePowers(localizationPacket, currentSequence.getMovement(sweepTimer.getSeconds()));
        if (sweepTimer.getSeconds() > currentSequence.getLastTime()) currentSequence = null;
        return driveTrainMovement;
    }

    /**
     * Advances action scheduling in simulation without calculating drive powers.
     * @param localizationPacket current robot localization in inches and degrees
     * @return action class name returned by {@link Sequence#updateActionsInSimulation(LocalizationPacket, double)}, or an empty string when no action starts
     */
    public String updateSimulation(LocalizationPacket localizationPacket){
        return currentSequence.updateActionsInSimulation(localizationPacket, sweepTimer.getSeconds());
    }
    /**
     * Gets the current target position.
     * @return target position in inches and heading in degrees
     */
    public Pos2D getRobotTargetPosition(){
        return getPathPoint().position;
    }
    /**
     * Gets the current target translation velocity.
     * @return target velocity in inches per second
     */
    public Pos2D getRobotTargetVelocity(){
        return currentSequence.getMovement(sweepTimer.getSeconds()).velocity;
    }
    /**
     * Gets the current target state, or a zero state while idle.
     * @return target position in inches and degrees, velocity in inches per second, and time in seconds
     */
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
    /**
     * Checks whether a sequence is currently active.
     * @return {@code true} when a sequence is being followed
     */
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
