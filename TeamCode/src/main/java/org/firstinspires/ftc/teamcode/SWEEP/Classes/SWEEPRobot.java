package org.firstinspires.ftc.teamcode.SWEEP.Classes;

import org.firstinspires.ftc.teamcode.SWEEP.Builder.Path;

/**
 * SWEEPRobot class is an abstract class that extends the Robot class and provides additional functionality
 * for autonomous path following. It includes methods for setting and following paths,
 * as well as default action ranges and times for triggering actions during autonomous operation.
 */
public abstract class SWEEPRobot extends Robot {
    // Current path being followed by the robot
    private Path currentPath;
    // Default range and time for actions to be triggered during autonomous operation
    private double defaultActionRange = 6; // Default range for actions to be triggered
    private double defaultActionTime = 0; // Default time for actions to be triggered

    /**
     * This method is called during the autonomous phase of the robot's operation.
     */
    @Override
    public void autonomousUpdate(){
        //Localization update
        //Other sweep logic
    }

    /**
     * Sets the current path for the robot to follow and starts the path.
     * @param path the path for the robot to follow
     */
    public void setPath(Path path){
        currentPath = path;
        path.start();
    }
    /**
     * Follows the current path set for the robot. This method should be called repeatedly during the autonomous phase.
     * It handles the logic for moving the robot along the path and executing any actions associated with the path.
     */
    public void followPath(){

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
