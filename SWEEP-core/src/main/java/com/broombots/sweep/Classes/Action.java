// Copyright Tobin Rumsey 2026 (c) all rights reserved.
package com.broombots.sweep.Classes;

/**
 * A generic framework for how actions should be structured.
 */
public abstract class Action {
    // The robot that this action is being executed on
    Robot robot;
    /** True once an interrupt has been accepted for the current run, so end() can switch to abort behaviour. */
    boolean interrupting = false;
    double waitTime = 0;
    double holdTime = 0; // Default hold time of 0 seconds, can be changed in the constructor
    // Timer to track how long the action has been running
    private final SWEEPTimer timer = new SWEEPTimer();
    /** True once end() has run for the current run, guarantees cleanup code is never executed twice. */
    private boolean ended = false;
    /** Seconds an interrupted action may spend recovering before it is force released, 0 or less means no limit. */
    private double recoveryTimeout = 0;
    /** Measures how long the current interrupt has been recovering, restarted when an interrupt is first accepted. */
    private final SWEEPTimer recoveryTimer = new SWEEPTimer();

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
    public void setWaitTime(double waitTime){
        this.waitTime = waitTime;
    }
    /**
     * Restarts the hold timer, called by whatever starts the action so hold time counts from the start, not construction.
     */
    public void resetTimer(){
        timer.reset();
        interrupting = false;
        ended = false;
    }
    /**
     * Lets an implementation change its behaviour when it is being stopped early.
     * Check this inside end() (and process() if interrupt() is overridden to block) to skip finishing moves
     * and put the mechanism in a safe state instead.
     * @return true if an interrupt has been accepted for the current run
     */
    protected final boolean isInterrupted(){
        return interrupting;
    }
    /**
     * Runs end() at most once per run, called by whatever finishes the action.
     * Public so that other packages that run actions (such as a Sequence) can finish them without risking a second end().
     */
    public void finish(){
        if (ended) return;
        ended = true;
        end();
    }
    /**
     * @return true if end() has already run for the current run
     */
    boolean hasEnded(){
        return ended;
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
     * If another action is queued that conflicts with this action, stop this action safely.
     * Returns true if the interrupt is accepted, false if there is a reason to block the change.
     * <p>
     * An accepted interrupt sets the interrupting flag (see {@link #isInterrupted()}). If the action is already
     * {@link #isSafeToRelease() safe to release} it ends immediately. Otherwise it stays active: process() keeps
     * running every loop so it can move the mechanism to a safe state, and end() only runs once
     * isSafeToRelease() returns true.
     * <p>
     * Override this to block an interrupt (return false without calling super); the flag stays unset until
     * an interrupt is accepted. Prefer {@link Robot#interruptAction(Action)} so the action is also removed from the
     * robot's active list once it has ended.
     */
    public boolean interrupt(){
        if (!interrupting) recoveryTimer.reset();
        interrupting = true;
        if (canReleaseAfterInterrupt()) finish();
        return true;
    }
    /**
     * Override for mechanisms that need time to recover after an interrupt, for example an arm that must be back in
     * a stable position before control can be released. Only asked while the action is being interrupted;
     * natural completion() is ignored during that time so a finished hold timer cannot cut recovery short.
     * @return true once it is safe to call end() and drop the action, false to keep process() running
     */
    protected boolean isSafeToRelease(){
        return true;
    }
    /**
     * Optional safety limit for recovery, call it in your constructor. Off by default, so a recovering action waits
     * for isSafeToRelease() forever. When set, an interrupted action that is still not safe after this many seconds
     * is force released: end() runs and the action is removed. Check {@link #didRecoveryTimeOut()} in end() to
     * react, for example by cutting power to the mechanism.
     * @param seconds recovery time limit in seconds, 0 or less turns the limit off
     */
    public void setRecoveryTimeout(double seconds){
        this.recoveryTimeout = seconds;
    }
    /**
     * @return true if the interrupt has been recovering for longer than the configured timeout
     */
    protected final boolean didRecoveryTimeOut(){
        return interrupting && recoveryTimeout > 0 && recoveryTimer.getSeconds() >= recoveryTimeout;
    }
    /**
     * @return true if the action can be released now, either because it reports safe or its recovery timed out
     */
    boolean canReleaseAfterInterrupt(){
        return isSafeToRelease() || didRecoveryTimeOut();
    }
    /**
     * Used by the action manager to check if the action is naturally complete by specific conditions
     * @return True if action is complete, False if it is still running
     */
    public boolean completion(){
        return timer.getSeconds() >= holdTime;
    }

    /**
     * The last bit of code that will run once the action is completed naturally
     */
    abstract public void end();

}
