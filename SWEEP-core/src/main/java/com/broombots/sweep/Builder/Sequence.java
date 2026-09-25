package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPAction;

import java.util.ArrayList;
import java.util.Collections;

/**
 * A Path is a collection of Segments that define a path for the robot to follow.
 * It also contains a list of actions that can be executed at specific times during the path.
 * A Path will have the finalized "animation" that the robot will follow, and then be passed to the movement controller to execute the path.
 */
public class Sequence {
    // The path that takes time and returns a Movement point, which provides the Pos2D position, velocity, and acceleration that the robot should be at that time
    private final ArrayList<PathPoint> compiledPath;

    // The actions that can be executed during the path, based on the position of the robot.
    private final ArrayList<SWEEPAction> actions;

    // The action that is currently being executed. This is used to determine if the action has completed and if the next action should be executed.
    private SWEEPAction activeAction;

    /**
     * Constructs a Path with the given segments and actions.
     * @param compiledPath the velocityMap that makes up the entire path - this will be based off of the robot Params
     * @param actions The actions that can be executed during the path.
     */
    public Sequence(ArrayList<PathPoint> compiledPath, SWEEPAction[] actions){
        if (compiledPath == null) throw new IllegalArgumentException("null path given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        this.compiledPath = compiledPath;
        this.actions = new ArrayList<>();
        Collections.addAll(this.actions, actions);

        System.out.println("Compiled path has " + compiledPath.size() + " points");
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

    private int[] getClosestIdxToTime(double time){
        int low = 0;
        int high = compiledPath.size();

        while (high - low > 1){
            int mid = (low+high) / 2;
            if (time >= compiledPath.get(mid).time){
                low = mid;
            } else {
                high = mid;
            }
        }

        return new int[]{low, high};
    }
    private double getRatioBetweenTimes(double t1, double t2, double targetTime){
        return (targetTime-t1)/(t2-t1);
    }
    private PathPoint lerpPathPoint(double ratio, PathPoint p1, PathPoint p2){
        PathPoint result = new PathPoint();
        result.time = lerp(p1.time, p2.time, ratio);
        result.position = Pos2D.lerpPos2D(ratio, p1.position, p2.position);
        result.velocity = Pos2D.lerpPos2D(ratio, p1.velocity, p2.velocity);
        return result;
    }

    public Pos2D getPosition(double time){
        return getMovement(time).position;
    }
    public PathPoint getMovement(double time){
        int[] closestIdx = getClosestIdxToTime(time);
        PathPoint p1 = compiledPath.get(closestIdx[0]);
        PathPoint p2 = compiledPath.get(closestIdx[1]);
        return lerpPathPoint(getRatioBetweenTimes(p1.time, p2.time, time), p1, p2);
    }
    public double getLastTime(){
        return compiledPath.get(compiledPath.size()-1).time;
    }
    private static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (end-start) * xInRange;
    }

}
