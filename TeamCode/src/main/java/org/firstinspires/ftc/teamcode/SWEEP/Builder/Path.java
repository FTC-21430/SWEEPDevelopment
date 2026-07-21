package org.firstinspires.ftc.teamcode.SWEEP.Builder;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.LocalizationPacket;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.RobotMovementParameters;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.SWEEPAction;

import java.util.ArrayList;
import java.util.Collections;

/**
 * A Path is a collection of Segments that define a path for the robot to follow.
 * It also contains a list of actions that can be executed at specific times during the path.
 * A Path will have the finalized "animation" that the robot will follow, and then be passed to the movement controller to execute the path.
 */
public class Path {
    // The path that takes time and returns a "Velocity" point, which provides the Coordinate and velocity that the robot should be at that time
    private final MovementMap compiledPath;
    // robotParams is the RobotMovementParameters interface which has everything about how the robot can move
    private final RobotMovementParameters robotParams;
    // The actions that can be executed during the path, based on the position of the robot.
    private final ArrayList<SWEEPAction> actions;
    //The index of the current segment that the robot is on. This is used to optimize the search for the current segment.
    private int currentSegmentIndex = 0;

    private final double startTime, endTime;

    // The action that is currently being executed. This is used to determine if the action has completed and if the next action should be executed.
    private SWEEPAction activeAction;

    /**
     * Constructs a Path with the given segments and actions.
     * @param compiledPath the velocityMap that makes up the entire path - this will be based off of the robot Params
     * @param robotParams the parameters of the robots drivetrain and how to can move. Used by this class to apply velocity data to motor powers
     * @param actions The actions that can be executed during the path.
     */
    public Path(MovementMap compiledPath, RobotMovementParameters robotParams, SWEEPAction[] actions, double startTime, double endTime){
        if (compiledPath == null) throw new IllegalArgumentException("null path given");
        if (robotParams == null) throw new IllegalArgumentException("null robot parameters given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        this.compiledPath = compiledPath;
        this.robotParams = robotParams
        this.actions = new ArrayList<>();
        this.startTime = startTime;
        this.endTime = endTime;
        Collections.addAll(this.actions, actions);
    }

    /**
     * Check the first action in queue based on robot location data, and execute it if met.
     * Shifts the queue forward if the front is executed
     * @param packet The localization packet containing the robot's current position and state.
     */
    public void updateActions(LocalizationPacket packet){
        if (activeAction != null){
            if (activeAction.completion()) {
                activeAction.end();
                activeAction = null;
            }else{
                activeAction.process();
            }
        } else if (!actions.isEmpty() && actions.get(0).checkTrigger(packet)){
            if (actions.isEmpty()) return;
            activeAction = actions.get(0);
            actions.remove(0);
            activeAction.execute();
        }
    }

    /**
     * Gets the total time of the path by subtracting the start time from the end time.
     * @return The total time of the path.
     */
    public double getTotalTime(){
        return getEndTime() - getStartTime();
    }
    /**
     * Gets the end time of the path by getting the end time of the last segment.
     * @return The end time of the path.
     */
    public double getEndTime(){
        return
    }
    /**
     * Gets the start time of the path by getting the start time of the first segment.
     * @return The start time of the path.
     */
    public double getStartTime(){
        return segments[0].getStartTime();
    }
    /**
     * Gets the position of the robot at a specific time.
     * @param time The time at which to get the position.
     * @return The position of the robot at the specified time.
     */
    public Coordinate getPosition(double time){
        return getCurrentSegment(time).getPosition(time);
    }
    /**
     * Gets the velocity of the robot at a specific time.
     * @param time The time at which to get the velocity.
     * @return The velocity of the robot at the specified time.
     */
    public double[] getVelocity(double time){
        return new double[]{0,0};
    }

}
