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

    private ArrayList<Double> segmentTimeMarkers;

    /**
     * Constructs a Path with the given segments and actions.
     * @param compiledPath the velocityMap that makes up the entire path - this will be based off of the robot Params
     * @param actions The actions that can be executed during the path.
     */
    public Sequence(ArrayList<PathPoint> compiledPath, SWEEPAction[] actions, ArrayList<Double> segmentTimeMarkers){
        if (compiledPath == null) throw new IllegalArgumentException("null path given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        if (segmentTimeMarkers == null) throw new IllegalArgumentException("null segmentTimeMarkers given");
        this.compiledPath = compiledPath;
        this.actions = new ArrayList<>();
        this.segmentTimeMarkers = segmentTimeMarkers;
        Collections.addAll(this.actions, actions);
        System.out.println("Compiled path has " + compiledPath.size() + " points");
    }

    /**
     * Adds the sequence built by the given SequenceBuilder to this sequence.
     * The idea is that you can add additional movement onto the sequence you are currently running!
     * The builder's own start point is ignored: the new path begins at the last point of this sequence
     * (position, velocity and time), so the combined path is always continuous.
     * @param sequenceBuilder the SequenceBuilder used to build the new sequence
     */
    public void addToSequence(SequenceBuilder sequenceBuilder){
        if (sequenceBuilder == null) throw new IllegalArgumentException("null sequenceBuilder given");
        if (compiledPath.isEmpty()) throw new IllegalStateException("Cannot append to a sequence with no path points");
        sequenceBuilder.continueFrom(compiledPath.get(compiledPath.size()-1));
        Sequence newSequence = sequenceBuilder.build();
        // the new actions' spline indices are relative to the new markers, so shift them past the existing ones
        for (SWEEPAction action : newSequence.actions){
            action.setSplineIndex(action.getSplineIndex() + segmentTimeMarkers.size());
        }
        this.compiledPath.addAll(newSequence.compiledPath);
        this.actions.addAll(newSequence.actions);
        this.segmentTimeMarkers.addAll(newSequence.segmentTimeMarkers);
    }

    /**
     * Check the first action in queue based on robot location data, and execute it if met.
     * Shifts the queue forward if the front is executed
     * @param packet The localization packet containing the robot's current position and state.
     * @param currentTime Seconds since the sequence started, actions wait until this passes their wait time.
     */
    public void updateActions(LocalizationPacket packet, double currentTime){
        if (activeAction != null){
            if (activeAction.completion()) {
                activeAction.finish();
                activeAction = null;
            }else{
                activeAction.process();
            }
        } else if (!actions.isEmpty() && actions.get(0).checkTrigger(packet, currentTime) && getTimeForSplineIndex(actions.get(0).getSplineIndex()) <= currentTime){
            if (actions.isEmpty()) return;
            activeAction = actions.get(0);
            actions.remove(0);
            activeAction.resetTimer();
            activeAction.execute();
        }
    }
    public String updateActionsInSimulation(LocalizationPacket packet, double currentTime){
        if (activeAction != null){
            if (activeAction.completion()) {
                activeAction = null;
            }
        } else if (!actions.isEmpty() && actions.get(0).checkTrigger(packet, currentTime) && getTimeForSplineIndex(actions.get(0).getSplineIndex()) <= currentTime){
            if (actions.isEmpty()) return "";
            activeAction = actions.get(0);
            actions.remove(0);
            return activeAction.getClass().toString();
        }
        return "";
    }

    private int[] getClosestIdxToTime(double time){
        if (compiledPath.isEmpty()) return new int[]{0, 0};
        int low = 0;
        int high = compiledPath.size()-1;

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
    private double getTimeForSplineIndex(int index){
        if (segmentTimeMarkers.isEmpty()) return 0;
        index = Math.max(0, Math.min(index, segmentTimeMarkers.size() - 1));
        return segmentTimeMarkers.get(index);
    }

    private int getCurrentSplineIdxToTime(double time){
        if (segmentTimeMarkers.isEmpty()) return 0;
        int low = 0;
        int high = segmentTimeMarkers.size()-1;

        while (high - low > 1){
            int mid = (low+high) / 2;
            if (time >= segmentTimeMarkers.get(mid)){
                low = mid;
            } else {
                high = mid;
            }
        }

        return low;
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
