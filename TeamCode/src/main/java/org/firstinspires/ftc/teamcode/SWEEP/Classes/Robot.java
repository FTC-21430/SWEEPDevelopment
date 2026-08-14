package org.firstinspires.ftc.teamcode.SWEEP.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * The Robot class is the base class for all robots in the SWEEP framework.
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
    public Alliance getAlliance(){
        return this.alliance;
    }

    /**
     * Sets the alliance for the robot
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance){
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
    public void emergencyStop(){
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
    public Subsystem[] getAllSubsystems(){
        return subsystems;
    }
    /**
     * Updates all active actions. This method should be called in the main update loop of the robot.
     * It processes each active action, checks for completion, and removes completed actions from the active list.
     */
    public void updateActions(){
        for (Action action : activeActions){
            action.process();
            if (action.completion()){
                action.end();
                activeActions.remove(action);
            }
        }
    }
    /**
     * Runs a given action. This method should be called to start an action and add it to the list of active actions.
     * @param action the action to be executed
     */
    public void runAction(Action action){
        if (action == null) return;
        action.execute();
        activeActions.add(action);
    }
}
