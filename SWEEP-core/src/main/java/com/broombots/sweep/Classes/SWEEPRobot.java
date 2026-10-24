package com.broombots.sweep.Classes;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.broombots.sweep.Movement.SequenceInterpreter;

/**
 * SWEEPRobot class is an abstract class that extends the Robot class and provides additional functionality
 * for autonomous path following. It includes methods for setting and following paths,
 * as well as default action ranges and times for triggering actions during autonomous operation.
 */
public abstract class SWEEPRobot extends Robot {
    /** Localization source; public so SDK-style robot classes can assign their hardware-backed implementation. */
    public Localization localization;
    /** Movement limits and gains, public so robot subclasses can replace or tune the defaults. */
    public RobotMovementParameters robotMovementParameters = new DefaultRobotMovementParameters();
    /** Sequence executor; public so robot subclasses can initialize it with their drive controller. */
    public SequenceInterpreter sequenceInterpreter;
    /** Default distance tolerance for autonomous action triggers, in inches. */
    private double defaultActionRange = 6;
    /** Default elapsed sequence path time before autonomous actions may trigger, in seconds. */
    private double defaultActionTime = 0;

    /** Updates actions, localization, and the active autonomous sequence. */
    @Override
    public void autonomousUpdate(){
        updateActions();
        localization.update();
        if (sequenceInterpreter.hasActiveSequence()){
            sequenceInterpreter.update(localization.getLocalizationPacket());
        }
        // Subclasses are responsible for updating their drivetrain here.

        telemetry.update();
    }

    /**
     * Sets the current path for the robot to follow and starts the path.
     * @param sequence the path for the robot to follow
     */
    public void setPath(Sequence sequence){
        sequenceInterpreter.startPath(sequence);
    }
    /**
     * Appends more movement to the path the robot is currently following.
     * @param sequenceBuilder the builder describing the movement to append, it continues from the end of the current path
     */
    public void addToCurrentSequence(SequenceBuilder sequenceBuilder){
        sequenceInterpreter.addToCurrentSequence(sequenceBuilder);
    }
    /** Gets the default autonomous action trigger distance.
     * @return default distance in inches
     */
    public double getDefaultActionRange() {
        return defaultActionRange;
    }
    /** Gets the default autonomous action path-time gate.
     * @return default elapsed sequence path time in seconds
     */
    public double getDefaultActionTime() {
        return defaultActionTime;
    }
}
