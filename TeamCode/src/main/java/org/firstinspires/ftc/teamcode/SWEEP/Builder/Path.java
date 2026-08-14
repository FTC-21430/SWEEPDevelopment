package org.firstinspires.ftc.teamcode.SWEEP.Builder;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.LocalizationPacket;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.SWEEPAction;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;

import java.util.ArrayList;
import java.util.Collections;

/**
 * A Path is a collection of Segments that define a path for the robot to follow.
 * It also contains a list of actions that can be executed at specific times during the path.
 * A Path will have the finalized "animation" that the robot will follow, and then be passed to the movement controller to execute the path.
 */
public class Path {
    // The segments that make up the path
    private final Segment[] segments;
    // The actions that can be executed during the path, based on the position of the robot.
    private final ArrayList<SWEEPAction> actions;
    //The index of the current segment that the robot is on. This is used to optimize the search for the current segment.
    private int currentSegmentIndex = 0;
    // The action that is currently being executed. This is used to determine if the action has completed and if the next action should be executed.
    private SWEEPAction activeAction;

    /**
     * Constructs a Path with the given segments and actions.
     * @param segments The segments that make up the path.
     * @param actions The actions that can be executed during the path.
     */
    public Path(Segment[] segments, SWEEPAction[] actions){
        if (segments == null) throw new IllegalArgumentException("null path given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        this.segments = segments;
        this.actions = new ArrayList<>();
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
        return segments[segments.length-1].getEndTime();
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
    /**
     * Starts the path from the beginning.
     */
    public void start(){
        currentSegmentIndex = 0;
    }
    /**
     * Starts the path from a specific time.
     * @param startingTime The time from which to start the path.
     */
    public void start(double startingTime){
        if (startingTime < 0) throw new IllegalArgumentException("Provided Time cannot be negative");
        currentSegmentIndex = getSegmentIndexAt(startingTime);
    }

    /**
     * Gets the current segment of the path at a specific time.
     * @param time target time
     * @return The segment of the path at the specified time.
     */
    private Segment getCurrentSegment(double time){
        return segments[getSegmentIndexAt(time)];
    }

    /**
     * Gets the index of the segment that is active at a specific time.
     * @param time The time at which to get the segment index.
     * @return The index of the segment that is active at the specified time.
     */
    private int getSegmentIndexAt(double time){
        if (segments.length == 0) throw new IllegalArgumentException("EmptyPath");
        if (time < 0) throw new IllegalArgumentException("Provided Time cannot be negative");

        if (time < getStartTime() || time > getEndTime()) return -1; // Time outside of path definition

        //Common case that the segment we were last on is the same one
        if (segments[currentSegmentIndex].activeAt(time)) return currentSegmentIndex;
        // Other normal case that the path just moved forward onto the next segment
        if (segments.length > currentSegmentIndex+1){
            if (segments[currentSegmentIndex+1].activeAt(time)) return currentSegmentIndex+1;
        }
        // If neither of these are true, we will then run a linear search for the target segment
        for (int i = 0; i < segments.length; i++){
            if(segments[i].activeAt(time)) return i;
        }

        // if that fails then we have a big problem
        throw new RuntimeException("No segment could be solved for time: Library Bug");
    }
}
