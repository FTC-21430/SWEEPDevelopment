package com.broombots.sweep.Classes;

/**
 * An {@link Action} that can be scheduled at a pose on a {@link com.broombots.sweep.Builder.Sequence}.
 * Subclass it to define mechanism work in {@link #execute()}, {@link #process()}, {@link #completion()}, and {@link #end()}.
 * <p>A sequence starts the action only when the robot is within {@link #triggerTolerance} inches of
 * {@link #triggerPosition} (planar distance; heading is ignored), sequence path time has reached
 * {@link #waitTime} seconds, and sequence path time
 * has reached the marker for {@link #splineIndex}. The wait time is path time since the sequence began, not a
 * delay measured from action construction. The spline index identifies the segment after which the action was added;
 * the sequence waits until that segment's time marker. A trigger position can be provided in the constructor or
 * inherited from the previous waypoint by the sequence builder. Constructors without a position support that
 * builder-assigned trigger or immediate robot-managed/teleop execution.
 * <p>Subclassing example:
 * <pre>{@code
 * public class IntakeAction extends SWEEPAction {
 *     public IntakeAction(Robot robot, Pos2D position) {
 *         super(robot, position, 0.0, 4.0, 0.0);
 *     }
 *     @Override public void execute() { }
 *     @Override public void process() { }
 *     @Override public boolean completion() { return true; }
 *     @Override public void end() { }
 * }
 * }</pre>
 */
public abstract class SWEEPAction extends Action {
    /** Allowed distance from the target pose in inches. */
    double triggerTolerance = 6;
    /** Index of the segment marker that must be reached before this action can start; -1 means unset. */
    int splineIndex = -1;
    /** Target field position in inches and heading in degrees that triggers this action. */
    Pos2D triggerPosition;

    /** Creates a path-triggered action with an explicit pose, hold duration, tolerance, and path-time gate.
     * @param robot robot instance associated with this action
     * @param triggerPosition field pose that triggers the action, in inches and degrees
     * @param holdTime minimum active duration in seconds
     * @param triggerTolerance allowed distance from trigger position in inches
     * @param waitTime sequence path time that must elapse before triggering, in seconds
     */
    public SWEEPAction(Robot robot, Pos2D triggerPosition, double holdTime, double triggerTolerance, double waitTime) {
        super(robot);
        this.triggerPosition = triggerPosition;
        setHoldTime(holdTime);
        this.triggerTolerance = triggerTolerance;
        setWaitTime(waitTime);
    }

    /** Creates an action with a hold duration and no explicit trigger pose.
     * @param robot robot instance associated with this action
     * @param holdTime minimum active duration in seconds
     */
    public SWEEPAction(Robot robot, double holdTime) {
        super(robot);
        setHoldTime(holdTime);
    }

    /** Creates an action with hold duration and sequence path-time gate, but no explicit trigger pose.
     * @param robot robot instance associated with this action
     * @param holdTime minimum active duration in seconds
     * @param waitTime sequence path time that must elapse before triggering, in seconds
     */
    public SWEEPAction(Robot robot, double holdTime, double waitTime) {
        super(robot);
        setHoldTime(holdTime);
        setWaitTime(waitTime);
    }

    /** Creates an action with default hold time and no explicit trigger pose.
     * @param robot robot instance associated with this action
     */
    public SWEEPAction(Robot robot) {
        super(robot);
    }

    /** Checks whether a trigger pose is currently set.
     * @return {@code true} if a trigger position has been assigned */
    public boolean isPositionSet() {
        return triggerPosition != null;
    }

    /** Sets the field position that triggers this action.
     * @param position trigger pose in inches and degrees
     * @throws NullPointerException if {@code position} is {@code null}
     */
    public void setPosition(Pos2D position) {
        if (position == null) throw new NullPointerException("Position cannot be null");
        this.triggerPosition = position;
    }

    /** Checks pose proximity and sequence path-time eligibility, excluding the spline marker check.
     * @param packet current robot pose
     * @param currentTime sequence elapsed path time in seconds
     * @return {@code true} if the robot is within tolerance and the configured wait time has elapsed
     */
    public boolean checkTrigger(LocalizationPacket packet, double currentTime) {
        return Pos2D.getDistanceBetweenCoordinates(packet.getCoordinate(), triggerPosition) <= triggerTolerance && currentTime >= waitTime;
    }

    /** Checks the trigger pose using zero sequence path time.
     * @param packet current robot pose
     * @return {@code true} if the robot is within tolerance and the configured wait time has elapsed
     */
    public boolean checkTrigger(LocalizationPacket packet) {
        return checkTrigger(packet, 0);
    }

    /** Checks whether a spline marker index has been assigned.
     * @return {@code true} if the index differs from the unset value {@code -1} */
    public boolean isSplineIndexSet() {
        return splineIndex != -1;
    }

    /** Returns the segment marker index associated with this action.
     * @return spline segment index, or {@code -1} if unset */
    public int getSplineIndex() {
        return splineIndex;
    }

    /** Assigns the segment marker index that gates this action in a sequence.
     * @param splineIndex zero-based segment marker index; {@code -1} represents unset
     */
    public void setSplineIndex(int splineIndex) {
        this.splineIndex = splineIndex;
    }
}
