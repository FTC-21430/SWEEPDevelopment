package org.firstinspires.ftc.teamcode.SWEEP.Classes;

/**
 * A class used to safely manage a robot's mechanisms with season specific code to the
 * generalized SWEEP Library, where the actions can be called autonomously
 *
 * //TODO: explain how to implement this system and write example code for using it
 */
public abstract class SWEEPAction extends Action {
    /**
     * Allowed trigger distance from the target position in inches.
     */
    double triggerTolerance = 6; //default tolerance of 6 inches, can be changed in the constructor
    /**
     * Target position in inches that triggers this action.
     */
    Coordinate triggerPosition;

    /**
     * Creates a new SWEEPAction, which extends an Action and adds the ability to be autonomously triggered with a route
     * @param robot the robot instance
     * @param triggerPosition the position that triggers this action
     * @param holdTime the time to hold the action
     * @param triggerTolerance the allowed trigger distance from the target position
     */
    public SWEEPAction(Robot robot, Coordinate triggerPosition, double holdTime, double triggerTolerance){
        super(robot);
        this.triggerPosition = triggerPosition;
        setHoldTime(holdTime);
        this.triggerTolerance = triggerTolerance;
    }

    /**
     * A simple sweep action that will use default triggers based on the PathBuilder or triggered immediately in teleop
     * @param robot the robot instance
     */
    public SWEEPAction(Robot robot){
        super(robot);
    }

    /**
     * Check if the trigger position has been set, used by the PathBuilder to know if it should override it with the previous position the user programmed in
     * @return true if the trigger position has been set, false otherwise
     */
    public boolean isPositionSet(){
        return triggerPosition != null;
    }
    /**
     * Set the trigger position for this action
     * @param position the position that triggers this action
     */
    public void setPosition(Coordinate position){
        if (position == null) throw new NullPointerException("Position cannot be null");
        this.triggerPosition = position;
    }
    /**
     * Check if the action is ready to trigger yet
     * @param packet Check proximity to the stored point
     * @return true if the action is ready to trigger, false otherwise
     */
    public boolean checkTrigger(LocalizationPacket packet){
        return Coordinate.getDistanceBetweenCoordinates(packet.getCoordinate(), triggerPosition) <= triggerTolerance;
    }
}
