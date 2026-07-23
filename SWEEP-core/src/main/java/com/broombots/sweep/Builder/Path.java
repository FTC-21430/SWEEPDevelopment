package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.SWEEPAction;

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

    // The action that is currently being executed. This is used to determine if the action has completed and if the next action should be executed.
    private SWEEPAction activeAction;

    /**
     * Constructs a Path with the given segments and actions.
     * @param compiledPath the velocityMap that makes up the entire path - this will be based off of the robot Params
     * @param robotParams the parameters of the robots drivetrain and how to can move. Used by this class to apply velocity data to motor powers
     * @param actions The actions that can be executed during the path.
     */
    public Path(MovementMap compiledPath, RobotMovementParameters robotParams, SWEEPAction[] actions){
        if (compiledPath == null) throw new IllegalArgumentException("null path given");
        if (robotParams == null) throw new IllegalArgumentException("null robot parameters given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        this.compiledPath = compiledPath;
        this.robotParams = robotParams;
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
    public String updateActionsInSimulation(LocalizationPacket packet){
        if (activeAction != null){
            if (activeAction.completion()) {
                activeAction = null;
            }else{
            }
        } else if (!actions.isEmpty() && actions.get(0).checkTrigger(packet)){
            if (actions.isEmpty()) return "";
            activeAction = actions.get(0);
            actions.remove(0);
            return activeAction.getClass().toString();
        }
        return "";
    }

    public Coordinate getPosition(double time){
        return getMovement(time).getPosition();
    }
    public MovementPoint getMovement(double time){
        return new MovementPoint(new Coordinate(0,0,0),0,0,0,0,0,0);
    }

}
