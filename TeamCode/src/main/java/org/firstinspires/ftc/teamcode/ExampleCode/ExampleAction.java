package org.firstinspires.ftc.teamcode.ExampleCode;

import org.firstinspires.ftc.teamcode.Robot.TemplateBot;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPAction;
/**
 * Template for implementing a new SWEEP action.
 * Override the lifecycle methods to add the action's trigger-time work, processing, completion, and cleanup.
 */
public class ExampleAction extends SWEEPAction {
    /**
     * Creates an example action that triggers around a coordinate and can run for a set amount of time.
     *
     * @param robot robot whose action scheduler will run this action
     * @param coordinate trigger center position in inches
     * @param time action runtime in seconds
     * @param range trigger radius in inches
     * @param waitTime path time in seconds that must pass before the action can trigger
     */
    public ExampleAction (TemplateBot robot, Pos2D coordinate, double time, double range, double waitTime){
        super(robot, coordinate, time, range, waitTime);
    }
    /** Starts the action when its trigger conditions are met. */
    @Override
    public void execute() {
        // This is where you would put the code that you want to run when this action is executed
    }
    /** Performs one iteration of action processing. */
    @Override
    public void process(){
        // This is where you would put the code that you want to iterate while this action is being processed
    }
    /**
     * Reports whether the action has completed.
     * @return true when the action is complete
     */
    @Override public boolean completion(){
        // This is where you would put the code that you want to run to check if this action is complete
        return true; // Action is complete
    }
    /** Performs cleanup when the action completes or is interrupted. */
    @Override public void end(){
        // This is where you would put the code that you want to run when this action is completed
        // isInterrupted() is true when the action was stopped early, use it to skip finishing moves and make things safe
    }
}
