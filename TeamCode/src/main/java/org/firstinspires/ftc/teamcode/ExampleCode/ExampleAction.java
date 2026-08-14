package org.firstinspires.ftc.teamcode.ExampleCode;

import org.firstinspires.ftc.teamcode.Robot.SoftwareTestingBot;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.SWEEPAction;
/**
 * Example action that can be triggered around a coordinate and run for a set amount of time.
 */
public class ExampleAction extends SWEEPAction {
    /**
     * Creates an example action that triggers around a coordinate and can run for a set amount of time.
     *
     * @param coordinate trigger center coordinate
     * @param time action runtime in seconds
     * @param range trigger radius in inches
     */
    public ExampleAction (SoftwareTestingBot robot, Coordinate coordinate, double time, double range){
        super(robot, coordinate, time, range);
    }
    @Override
    public void execute() {
        // This is where you would put the code that you want to run when this action is executed
    }
    @Override
    public void process(){
        // This is where you would put the code that you want to iterate while this action is being processed
    }
    @Override public boolean completion(){
        // This is where you would put the code that you want to run to check if this action is complete
        return true; // Action is complete
    }
    @Override public void end(){
        // This is where you would put the code that you want to run when this action is completed
    }
}
