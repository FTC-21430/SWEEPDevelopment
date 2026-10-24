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
    /** Driver Station telemetry; public so SDK-style robot classes can assign the SDK-provided instance. */
    public Telemetry telemetry;
    /** Robot subsystems; public so robot subclasses can assign their subsystem collection. */
    public Subsystem[] subsystems;
    /** Match runtime timer in seconds; public so SDK-style robot classes can assign the SDK-provided timer. */
    public ElapsedTime runtime;
    /** Current alliance; package-visible for robot implementations in this package. */
    Alliance alliance = Alliance.RED;
    /** Active actions; package-visible for package-level robot lifecycle tests. */
    ArrayList<Action> activeActions = new ArrayList<>();
    /** Alliance choices returned by the protected alliance accessors. */
    enum Alliance {
        /** Red alliance. */
        RED,
        /** Blue alliance. */
        BLUE
    }

    /**
     * Initializes the robot.
     * This method should be overridden by subclasses to provide specific initialization logic for the robot.
     */
    public void initialize(){

    }

    /** Resets the match runtime timer. */
    public void startMatch(){
        runtime.reset();
    }

    /** Returns the alliance assigned to this robot.
     * @return current alliance
     */
    protected Alliance getAlliance(){
        return this.alliance;
    }

    /** Sets the alliance for the robot.
     * @param alliance alliance to assign
     */
    protected void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    /** Updates the robot during autonomous operation; override to provide robot-specific autonomous behavior. */
    public void autonomousUpdate(){
        telemetry.addLine("Autonomous in progress, override autonomousUpdate() in your robot to customize it.");
        updateActions();
        telemetry.update();
    }
    /** Updates the robot during teleop operation; override to provide robot-specific driver-controlled behavior. */
    public void teleopUpdate(){
        telemetry.addLine("Teleop in progress, override teleopUpdate() in your robot to customize it.");
        updateActions();
        telemetry.update();
    }

    /** Disables every subsystem and reports an emergency-stop message through telemetry. */
    protected void emergencyStop(){
        for (Subsystem subsystem : subsystems){
            subsystem.setDisabled(true);
        }
        telemetry.addLine("--ROBOT EMERGENCY STOPPED, RE INIT TO CONTINUE--");
        telemetry.update();
    }
    /** Returns elapsed match time.
     * @return game time in seconds since {@link #startMatch()}
     */
    public double getGameTime(){
        return runtime.seconds();
    }
    /** Returns all subsystems assigned to this robot.
     * @return robot subsystems
     */
    protected Subsystem[] getAllSubsystems(){
        return subsystems;
    }
    /** Processes active actions, finishes completed ones, and removes them from the active list. */
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
    /** Starts a non-null action and adds it to the active list.
     * @param action action to execute
     */
    public void runAction(Action action){
        if (action == null) return;
        action.resetTimer();
        action.execute();
        activeActions.add(action);
    }
}
