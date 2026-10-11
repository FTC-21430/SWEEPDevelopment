package com.broombots.sweep.Classes;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Movement.SequenceInterpreter;

/**
 * SWEEPRobot class is an abstract class that extends the Robot class and provides additional functionality
 * for autonomous path following. It includes methods for setting and following paths,
 * as well as default action ranges and times for triggering actions during autonomous operation.
 */
public abstract class SWEEPRobot extends Robot {
    public Localization localization;
    public RobotMovementParameters robotMovementParameters;
    public SequenceInterpreter sequenceInterpreter = new SequenceInterpreter(robotMovementParameters);
    // Default range and time for actions to be triggered during autonomous operation
    private double defaultActionRange = 6; // Default range for actions to be triggered
    private double defaultActionTime = 0; // Default time for actions to be triggered

    /**
     * This method is called during the autonomous phase of the robot's operation.
     */
    @Override
    public void autonomousUpdate(){
        updateActions();
        localization.update();
        if (sequenceInterpreter.hasActiveSequence()){
            sequenceInterpreter.update(localization.getLocalizationPacket());
        }
        // specific drivetrain update.... use in override

        //Localization update
        //Other sweep logic

        telemetry.update();
    }

    /**
     * Sets the current path for the robot to follow and starts the path.
     * @param sequence the path for the robot to follow
     */
    public void setPath(Sequence sequence){
        sequenceInterpreter.startPath(sequence);
    }
    public void addToCurrentSequence(Sequence sequence){
        sequenceInterpreter.addToCurrentSequence(sequence);
    }
    /**
     * Gets the default range for actions to be triggered during autonomous operation.
     * @return the default action range
     */
    public double getDefaultActionRange() {
        return defaultActionRange;
    }
    /**
     * Gets the default time for actions to be triggered during autonomous operation.
     * @return the default action time
     */
    public double getDefaultActionTime() {
        return defaultActionTime;
    }
}
