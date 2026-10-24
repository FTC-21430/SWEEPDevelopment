package com.broombots.sweep.Classes;

/** Defines a robot subsystem's periodic update and disabled-state behavior. */
public interface Subsystem {
    /** Updates this subsystem once during the robot's control loop. */
    public void update();
    /** Applies or clears disabled state; disabling must immediately stop motors and reset relevant state.
     * @param disable {@code true} to disable the subsystem, {@code false} to enable it
     */
    public void setDisabled(boolean disable);
}
