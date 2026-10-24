package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPAction;

import java.util.ArrayList;
import java.util.Collections;

/**
 * A compiled, time-parameterized robot path with its queued actions.
 * The sequence stores sampled poses and velocities for execution and schedules actions at their associated path times.
 */
public class Sequence {
    /** Sampled path poses, velocities, and times; positions are in inches, headings in degrees, velocities in inches per second, and times in seconds. */
    private final ArrayList<PathPoint> compiledPath;

    /** Actions scheduled to trigger during this sequence. */
    private final ArrayList<SWEEPAction> actions;

    /** Action currently being processed, or {@code null} when none is active. */
    private SWEEPAction activeAction;

    /** Start times in seconds indexed by waypoint segment index. */
    private final ArrayList<Double> segmentTimeMarkers;

    /**
     * Constructs a sequence from compiled path points, actions, and segment start times.
     * @param compiledPath sampled path states with positions in inches, headings in degrees, velocities in inches per second, and times in seconds
     * @param actions the actions that can be executed during the sequence
     * @param segmentTimeMarkers start times in seconds for each spline segment, indexed by waypoint segment index
     * @throws IllegalArgumentException if any argument is {@code null}
     */
    public Sequence(ArrayList<PathPoint> compiledPath, SWEEPAction[] actions, ArrayList<Double> segmentTimeMarkers){
        if (compiledPath == null) throw new IllegalArgumentException("null path given");
        if (actions == null) throw new IllegalArgumentException("null action array given");
        if (segmentTimeMarkers == null) throw new IllegalArgumentException("null segmentTimeMarkers given");
        this.compiledPath = compiledPath;
        this.actions = new ArrayList<>();
        this.segmentTimeMarkers = segmentTimeMarkers;
        Collections.addAll(this.actions, actions);
    }

    /**
     * Adds the sequence built by the given SequenceBuilder to this sequence.
     * Appends movement to the sequence currently being built or run.
     * The builder's own start point is ignored: the new path begins at the last point of this sequence
     * (position, velocity and time), so the combined path is always continuous.
     * @param sequenceBuilder the builder used to build the appended sequence
     * @throws IllegalArgumentException if {@code sequenceBuilder} is {@code null}
     * @throws IllegalStateException if this sequence has no path points
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
     * @param packet current robot localization with position in inches and heading in degrees
     * @param currentTime elapsed sequence time in seconds
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
    /**
     * Advances simulated action scheduling without executing the action.
     * @param packet current robot localization in inches and degrees
     * @param currentTime elapsed sequence time in seconds
     * @return the action class name string returned by {@code Class.toString()}, or an empty string if none would start
     */
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

    /** Finds the path sample indices surrounding the requested sequence time. */
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
    /** Returns the sequence time associated with a path sample index. */
    private double getTimeForSplineIndex(int index){
        if (segmentTimeMarkers.isEmpty()) return 0;
        index = Math.max(0, Math.min(index, segmentTimeMarkers.size() - 1));
        return segmentTimeMarkers.get(index);
    }


    /** Computes the interpolation ratio of a target time within a time interval. */
    private double getRatioBetweenTimes(double t1, double t2, double targetTime){
        return (targetTime-t1)/(t2-t1);
    }
    /** Interpolates all path-point values between two samples. */
    private PathPoint lerpPathPoint(double ratio, PathPoint p1, PathPoint p2){
        PathPoint result = new PathPoint();
        result.time = lerp(p1.time, p2.time, ratio);
        result.position = Pos2D.lerpPos2D(ratio, p1.position, p2.position);
        result.velocity = Pos2D.lerpPos2D(ratio, p1.velocity, p2.velocity);
        return result;
    }

    /**
     * Gets the interpolated position at a sequence time.
     * @param time sequence time in seconds
     * @return position in inches and heading in degrees; times outside the path are clamped to an endpoint
     */
    public Pos2D getPosition(double time){
        return getMovement(time).position;
    }
    /**
     * Gets the interpolated path state between the two nearest compiled points.
     * The interpolation ratio is clamped, so times outside the path resolve to the first or last point.
     * @param time sequence time in seconds
     * @return interpolated pose, velocity, and time
     */
    public PathPoint getMovement(double time){
        int[] closestIdx = getClosestIdxToTime(time);
        PathPoint p1 = compiledPath.get(closestIdx[0]);
        PathPoint p2 = compiledPath.get(closestIdx[1]);
        return lerpPathPoint(getRatioBetweenTimes(p1.time, p2.time, time), p1, p2);
    }
    /**
     * Gets the time of the final compiled path point.
     * @return final path time in seconds
     */
    public double getLastTime(){
        return compiledPath.get(compiledPath.size()-1).time;
    }
    /** Interpolates between two scalar values using the supplied ratio. */
    private static double lerp(double start, double end, double x){
        double xInRange = x < 0.0 ? 0.0 : Math.min(1.0, x); // keep x in range of 0.0-1.0
        return start + (end-start) * xInRange;
    }

}
