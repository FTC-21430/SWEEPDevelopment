package org.firstinspires.ftc.teamcode.SWEEP.Classes;
/**
 * Interface for localization systems. This interface defines the methods that any localization system must implement.
 * Localization systems are responsible for tracking the robot's position and orientation on the field.
 */
public interface Localization {
    /**
     * updates the localization data
     */
    public void update();
    /**
     * Retrieves the current localization data.
     * @return A LocalizationPacket containing the robot's current position and orientation.
     */
    public LocalizationPacket getLocalizationPacket();
    /**
     * Overrides the current localization data with the specified values.
     * @param x     X position, in inches
     * @param y     Y position, in inches
     * @param angle Heading angle, in degrees
     */
    public void overrideLocalization(double x, double y, double angle);
}
