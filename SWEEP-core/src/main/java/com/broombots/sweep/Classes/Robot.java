package com.broombots.sweep.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * The Robot class is the base class for all robots in the BroomBots framework.
 * It contains the basic structure and functionality that all robots should have,
 * including subsystems, actions, and telemetry.
 */
public abstract class Robot {
    // The telemetry object for sending data to the driver station (debug information)
    public Telemetry telemetry;
    // An array of subsystems that make up the robot
    public Subsystem[] subsystems;
    // The runtime timer for tracking the duration of the match
    public ElapsedTime runtime;
    // The alliance that the robot is currently on (RED or BLUE)
    Alliance alliance = Alliance.RED;
    // A list of actions that are currently active, list functionality is for teleop,
    // to store driver inputs that were given while an action was running.
    ArrayList<Action> activeActions = new ArrayList<>();
    /**
     * An Enum for the two different alliances that the robot could be a part of
     */
    enum Alliance {
        RED, BLUE
    }

    /**
     * Initializes the robot.
     * This method should be overridden by subclasses to provide specific initialization logic for the robot.
     */
    public void initialize(){

    }

    /**
     * Resets game timer
     */
    public void startMatch(){
        runtime.reset();
    }

    /**
     * @return the alliance the robot has been set to
     */
    protected Alliance getAlliance(){
        return this.alliance;
    }

    /**
     * Sets the alliance for the robot
     * @param alliance alliance
     */
    protected void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    /**
     * Updates the robot during the autonomous period.
     * This method should be overridden by subclasses to provide specific autonomous logic for the robot.
     */
    public void autonomousUpdate(){
        telemetry.addLine("Teleop In progress, Override the teleopUpdate method in robot to change something!");
        updateActions();
        telemetry.update();
    }
    /**
     * Updates the robot during the teleop period.
     * This method should be overridden by subclasses to provide specific teleop logic for the robot.
     */
    public void teleopUpdate(){
        telemetry.addLine("Autonomous In progress, Override the teleopUpdate method in robot to change something!");
        updateActions();
        telemetry.update();
    }

    /**
     * Stops all subsystems and displays an emergency stop message on the telemetry.
     * Ensure that all subsystems have their disabled functions implemented properly.
     */
    protected void emergencyStop(){
        for (Subsystem subsystem : subsystems){
            subsystem.setDisabled(true);
        }
        telemetry.addLine("--ROBOT EMERGENCY STOPPED, RE INIT TO CONTINUE--");
        telemetry.update();
    }
    /**
     * Returns the current game time in seconds since the match started.
     * @return game time
     */
    public double getGameTime(){
        return runtime.seconds();
    }
    /**
     * Returns all subsystems of the robot.
     * @return array of subsystems
     */
    protected Subsystem[] getAllSubsystems(){
        return subsystems;
    }
    /**
     * Updates all active actions. This method should be called in the main update loop of the robot.
     * It processes each active action, checks for completion, and removes completed actions from the active list.
     */
    protected void updateActions(){
        // Iterate a snapshot because process() and end() may start new actions, which modifies activeActions
        for (Action action : new ArrayList<>(activeActions)){
            if (action.hasEnded()){
                // interrupted directly through Action.interrupt(), end() already ran so only drop it
                activeActions.remove(action);
                continue;
            }
            action.process();
            // while interrupting, only the action's own recovery check (or its optional timeout) can release it
            boolean done = action.interrupting ? action.canReleaseAfterInterrupt() : action.completion();
            if (done){
                activeActions.remove(action);
                action.finish();
            }
        }
    }
    /**
     * Requests an interrupt of a running action. If it is accepted and the action is already safe to release it is
     * removed immediately; otherwise it stays active, keeps being processed while it recovers, and is removed by
     * updateActions() once its isSafeToRelease() returns true.
     * @param action the action to interrupt
     * @return true if the interrupt was accepted (the action may still be recovering), false if it blocked the
     * interrupt or is not running
     */
    public boolean interruptAction(Action action){
        if (action == null || !activeActions.contains(action)) return false;
        if (!action.interrupt()) return false;
        if (action.hasEnded()) activeActions.remove(action);
        return true;
    }
    /**
     * Runs a given action. This method should be called to start an action and add it to the list of active actions.
     * @param action the action to be executed
     */
    public void runAction(Action action){
        if (action == null) return;
        action.resetTimer();
        action.execute();
        activeActions.add(action);
    }
}
