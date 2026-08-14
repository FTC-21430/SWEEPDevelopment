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
    // The path that takes time and returns a Movement point, which provides the Coordinate, velocity, and acceleration that the robot should be at that time
    private final MovementMap compiledPath;

    // The actions that can be executed during the path, based on the position of the robot.
    private final ArrayList<SWEEPAction> actions;

    // The action that is currently being executed. This is used to determine if the action has completed and if the next action should be executed.
    private SWEEPAction activeAction;

    /**
     * Constructs a Path with the given segments and actions.
     * @param compiledPath the velocityMap that makes up the entire path - this will be based off of the robot Params
     * @param actions The actions that can be executed during the path.
     */
    public Path(MovementMap compiledPath, SWEEPAction[] actions){
        if (compiledPath == null) throw new IllegalArgumentException("null path given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        this.compiledPath = compiledPath;
        this.actions = new ArrayList<>();
        Collections.addAll(this.actions, actions);

        System.out.println("Compiled path has " + compiledPath.getAllPoints().size() + " points");
        System.out.println(compiledPath.getAllPoints());
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
        return compiledPath.getPoint(time);
    }
    public double getEndTime(){
        return compiledPath.getLastTime();
    }
}
