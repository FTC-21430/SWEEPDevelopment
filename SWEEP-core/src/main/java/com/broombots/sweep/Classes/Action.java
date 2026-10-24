// Copyright Tobin Rumsey 2026 (c) all rights reserved.
package com.broombots.sweep.Classes;

/**
 * Base class for robot actions with lifecycle, timing, and interruption support.
 * Subclasses implement the action work while {@link Robot} manages execution and cleanup.
 */
public abstract class Action {
    /** Robot running this action; package-visible for SWEEP action implementations. */
    Robot robot;
    /** Whether an interrupt was accepted for this run; package-visible to the robot lifecycle manager. */
    boolean interrupting = false;
    /** Minimum sequence path time in seconds before a SWEEP action may trigger; package-visible to its trigger checks. */
    double waitTime = 0;
    /** Minimum active duration in seconds; package-visible for action implementations sharing timing state. */
    double holdTime = 0;
    /** Measures elapsed action time in seconds. */
    private final SWEEPTimer timer = new SWEEPTimer();
    /** True once end() has run for the current run, guarantees cleanup code is never executed twice. */
    private boolean ended = false;
    /** Seconds an interrupted action may spend recovering before it is force released, 0 or less means no limit. */
    private double recoveryTimeout = 0;
    /** Measures interrupt recovery time in seconds, restarted when an interrupt is first accepted. */
    private final SWEEPTimer recoveryTimer = new SWEEPTimer();

    /** Creates an action associated with the given robot.
     * @param robot robot that will run this action
     */
    public Action(Robot robot){
        this.robot = robot;
    }

    /** Sets the minimum active duration before this action can complete naturally.
     * @param holdTime minimum duration in seconds
     */
    public void setHoldTime(double holdTime){
        this.holdTime = holdTime;
    }
    /** Sets the wait duration used by subclasses or schedulers before this action may start.
     * @param waitTime wait duration in seconds
     */
    public void setWaitTime(double waitTime){
        this.waitTime = waitTime;
    }
    /** Restarts this run's timer and clears lifecycle state before the action starts. */
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
    /** @return {@code true} if {@link #end()} has already run for this action run */
    boolean hasEnded(){
        return ended;
    }
    /** Performs one-time setup when the action starts. */
    public abstract void execute();

    /** Performs one iteration of the action while it is active. */
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
     * @return {@code true} if the interrupt was accepted, {@code false} if this action blocks it
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
     * @return {@code true} when it is safe to call {@link #end()} and release the action
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
    /** @return {@code true} if interrupt recovery has reached its configured timeout */
    protected final boolean didRecoveryTimeOut(){
        return interrupting && recoveryTimeout > 0 && recoveryTimer.getSeconds() >= recoveryTimeout;
    }
    /** @return {@code true} if the action is safe to release or its recovery timed out */
    boolean canReleaseAfterInterrupt(){
        return isSafeToRelease() || didRecoveryTimeOut();
    }
    /** Determines whether this action has completed naturally.
     * @return {@code true} when the action's hold duration has elapsed
     */
    public boolean completion(){
        return timer.getSeconds() >= holdTime;
    }

    /** Performs final cleanup when the action ends, naturally or after an accepted interrupt. */
    abstract public void end();

}
