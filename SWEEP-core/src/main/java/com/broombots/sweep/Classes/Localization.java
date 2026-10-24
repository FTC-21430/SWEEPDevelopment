package com.broombots.sweep.Classes;

/** Provides the robot's current field pose to SWEEP path following and control. */
public interface Localization {
    /** Updates the localization estimate; subsequent packet reads must reflect the refreshed estimate. */
    public void update();
    /** Returns the latest localization sample.
     * @return current robot pose and field-frame velocity; position is inches, heading degrees, linear velocity inches per second */
    public LocalizationPacket getLocalizationPacket();
    /** Replaces the current estimated pose with the supplied field coordinates.
     * @param x field x position in inches
     * @param y field y position in inches
     * @param angle field heading in degrees
     */
    public void overrideLocalization(double x, double y, double angle);
}
