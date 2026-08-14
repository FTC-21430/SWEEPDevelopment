package org.firstinspires.ftc.teamcode.SWEEP.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A generic framework for how actions should be structured.
 */
public abstract class Action {
    // The robot that this action is being executed on
    Robot robot;
    double holdTime = 0; // Default hold time of 0 seconds, can be changed in the constructor
    // Timer to track how long the action has been running
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Create a new action, called as a super,
     * @param robot The robot that this action is being executed on.
     */
    public Action(Robot robot){
        this.robot = robot;
    }

    /**
     * The hold time that the action should hold
     * @param holdTime The duration in seconds for which the action should hold.
     */
    public void setHoldTime(double holdTime){
        this.holdTime = holdTime;
    }
    /**
     * Start the action - any initial logic
     */
    public abstract void execute();

    /**
     * Update loop for the action, runs every master iteration
     */
    public abstract void process();
    /**
     * If another action is queued that conflicts with this action, stop this action safely
     * Returns true if the action is complete, will return false if there is a reason to block the change
     *
     */
    public boolean interrupt(){
        end();
        return true; // Action can be interrupted safely
    }
    /**
     * Used by the action manager to check if the action is naturally complete by specific conditions
     * @return True if action is complete, False if it is still running
     */
    public boolean completion(){
        return timer.seconds() >= holdTime;
    }

    /**
     * The last bit of code that will run once the action is completed naturally
     */
    abstract public void end();

}
