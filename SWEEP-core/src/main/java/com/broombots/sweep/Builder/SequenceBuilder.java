package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Classes.SWEEPAction;
import com.broombots.sweep.Classes.Waypoint;
import com.broombots.sweep.Splines.EndWaypoint;
import com.broombots.sweep.Splines.BreakWaypoint;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.Segments.AngledSplineSegment;
import com.broombots.sweep.Splines.Segments.FollowSplineSegment;
import com.broombots.sweep.Splines.Segments.WaitSegment;
import com.broombots.sweep.Splines.SplineAngleWaypoint;
import com.broombots.sweep.Splines.SplineOffsetWaypoint;
import com.broombots.sweep.Splines.SplineWaypoint;
import com.broombots.sweep.Splines.StartWaypoint;
import com.broombots.sweep.Splines.WaitWaypoint;

import java.util.ArrayList;

/**
 * PathBuilder is a class that allows for the creation of a path for a robot to follow.
 * It uses waypoints to define the path and actions to be executed at specific points along the path.
 * Class methods will build upon itself until the build() method is called, which will return a Path object that can be used to follow the defined path.
 */
public class SequenceBuilder {
    /** Field length used for coordinate clipping, in inches; coordinates are limited to +/-72 inches. */
    private final double fieldLength = 144; // inches
    /**
     * The array of all waypoints that define the path.
     * Waypoints are added to this array using the various splineTo and linearTo methods.
     */
    private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    /**
     * Actions scheduled to run at path positions, in the order they are added.
     */
    private final ArrayList<SWEEPAction> actions = new ArrayList<SWEEPAction>();
    /**
     * Last added coordinate, in inches and degrees; used as the default location for subsequent actions and waits.
     */
    private Pos2D previousCoordinate;
    /**
     * Step in spline parameter space (0 to 1) between samples; smaller steps increase resolution and compile time.
     * The default is 0.0005 parameter units per sample, not seconds.
     */
    private double sampleRate = 0.0005;
    /** Robot motion limits used to compile this sequence. */
    private RobotMovementParameters movementParameters;
    /** Starting state copied into the compiled path; position is in inches, heading in degrees, velocity in inches per second, and time in seconds. */
    private PathPoint startingPoint;
    /** Desired final velocity as a fraction of maximum velocity, from 0 to 1. */
    private double endingSpeedRatio = 0.0;
    /**
     * Constructs a new SequenceBuilder object.
     * Initializes the waypoints and actions arrays, and sets the previous coordinate to (0,0,0).
     * @param movementParameters the definitions of the physical movement capabilities of the robot that will follow this path.
     */
    public SequenceBuilder(RobotMovementParameters movementParameters){
        previousCoordinate = new Pos2D(0,0,0);
        this.movementParameters = movementParameters;
    }
    /**
     * Sets the spline parameter step used when sampling curves.
     * @param sampleRate parameter step from 0 to 1 per sample, not a time interval
     * @return this builder for method chaining
     */
    public SequenceBuilder setSampleRate(double sampleRate){
        this.sampleRate = sampleRate;
        return this;
    }

    /**
     * Clears accumulated waypoints and actions and restores the default origin start coordinate.
     * @return this builder for method chaining
     */
    public SequenceBuilder resetBuilder(){
        waypoints.clear();
        actions.clear();
        previousCoordinate = new Pos2D(0,0,0);
        endingSpeedRatio = 0.0;
        return this;
    }

    /**
     * Adds an action to the path.
     * If the action does not have a position set, it will be set to the previous coordinate.
     * Actions are executed in the order they are added to the path, and will be
     * triggered when the robot is within a certain distance of the action's trigger.
     * @param actionClass The action to be added to the path.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder addAction(SWEEPAction actionClass){
        if (actionClass == null) throw new NullPointerException("Action cannot be null");
        if (!actionClass.isPositionSet()) actionClass.setPosition(previousCoordinate);
        actionClass.setSplineIndex(waypoints.size()-1);
        actions.add(actionClass);

        return this;
    }

    /**
     * Adds a spline waypoint to the path. Robot heading will follow the curve of the spline
     * @param x x coordinate in inches.
     * @param y y coordinate in inches.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineTo(double x, double y, double speed){
        speed = Math.min(speed, 1);
        speed = Math.max(speed,0);
        waypoints.add(new SplineWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),speed));
        previousCoordinate = new Pos2D(x,y,Double.NaN);
        return this;
    }

    /**
     * Adds a spline waypoint to the path using a defined coordinate.
     * Robot heading will follow the curve of the spline.
     * @param definedCoordinate waypoint position in inches and heading in degrees.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineTo(Pos2D definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineWaypoint(definedCoordinate.x,definedCoordinate.y,speed));
        previousCoordinate = new Pos2D(definedCoordinate.x,definedCoordinate.y, Double.NaN); // Not a Number signals that the angle must dynamically set after the spline to segment.
        return this;
    }
    /**
     * Adds a spline waypoint to the path with a specified angle.
     * Robot heading will smoothly transition to the specified angle as it approaches the waypoint.
     * @param x x coordinate in inches.
     * @param y y coordinate in inches.
     * @param angle robot heading in degrees.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineToAngle(double x, double y, double angle, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineAngleWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),angle,speed));
        previousCoordinate = new Pos2D(x,y,angle);
        return this;
    }
    /**
     * Adds a spline waypoint to the path with a specified angle using a defined coordinate.
     * Robot heading will smoothly transition to the specified angle as it approaches the waypoint.
     * @param definedCoordinate waypoint position in inches and heading in degrees.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineToAngle(Pos2D definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineAngleWaypoint(definedCoordinate.x,definedCoordinate.y,definedCoordinate.angle,speed));
        previousCoordinate = definedCoordinate;
        return this;
    }
    /**
     * Adds a spline waypoint where the robot heading stays a fixed angle away from the direction of travel.
     * The path is the same curve as {@link #splineTo(double, double, double)}, only the heading differs.
     * Switching between forward and offset segments makes the heading jump at the boundary, so add a
     * {@link #addBreak()} or {@link #waitAt(double)} there to let the robot turn while stopped.
     * @param x The x coordinate of the waypoint, in inches.
     * @param y The y coordinate of the waypoint, in inches.
     * @param offsetDegrees heading offset from the direction of travel, in degrees; positive is counterclockwise.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineToOffset(double x, double y, double offsetDegrees, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineOffsetWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),offsetDegrees,speed));
        previousCoordinate = new Pos2D(x,y,Double.NaN); // Not a Number signals that the angle is resolved after the segment
        return this;
    }
    /**
     * Adds an offset spline waypoint using a defined coordinate, the angle of the coordinate is ignored.
     * @param definedCoordinate The coordinate of the waypoint, in inches.
     * @param offsetDegrees heading offset from the direction of travel, in degrees; positive is counterclockwise.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     * @see #splineToOffset(double, double, double, double)
     */
    public SequenceBuilder splineToOffset(Pos2D definedCoordinate, double offsetDegrees, double speed){
        return splineToOffset(definedCoordinate.x, definedCoordinate.y, offsetDegrees, speed);
    }
    /**
     * Adds a spline waypoint that the robot drives to backwards, with the rear of the robot leading.
     * Equivalent to {@code splineToOffset(x, y, 180, speed)}.
     * @param x The x coordinate of the waypoint, in inches.
     * @param y The y coordinate of the waypoint, in inches.
     * @param speed speed ratio from 0 to 1.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineToBackwards(double x, double y, double speed){
        return splineToOffset(x, y, 180, speed);
    }
    /**
     * Adds a break waypoint to the path.
     * break waypoints will force the robot to slow down and continue on the path, ignoring previous waypoint tangents and angles.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder addBreak(){
        waypoints.add(new BreakWaypoint(previousCoordinate));
        return this;
    }
    /**
     * Adds a wait waypoint to the path, similar to a break waypoint, but with a specified duration to wait at the previous waypoint before continuing on the path.
     * The robot will stop at the previous waypoint and wait for the specified duration before continuing on the path.
     * @param duration wait duration in seconds.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder waitAt(double duration){
        waypoints.add(new WaitWaypoint(previousCoordinate, duration));
        return this;
    }
    /**
     * Adds a start waypoint to the path, which must be the first waypoint in the path.
     * The robot will start at this waypoint and begin following the path from this point.
     * @param x start x coordinate in inches.
     * @param y start y coordinate in inches.
     * @param angle robot heading in degrees.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder start(double x, double y, double angle){
        PathPoint point = new PathPoint();
        point.time = 0;
        point.position = new Pos2D(x, y, angle);
        point.velocity = new Pos2D(0,0,0);
        start(point);
        return this;
    }
    /**
     * Sets the starting state of the sequence by copying the supplied point.
     * @param startingPoint initial position in inches, heading in degrees, velocity in inches per second, and time
     * @return this builder for method chaining
     */
    public SequenceBuilder start(PathPoint startingPoint){
        PathPoint copy = copyOf(startingPoint); // copied so building never changes the caller's point
        copy.time = 0;
        waypoints.add(new StartWaypoint(copy.position.x, copy.position.y, copy.position.angle));
        previousCoordinate = new Pos2D(copy.position.x, copy.position.y, copy.position.angle);
        this.startingPoint = copy;
        return this;
    }
    /**
     * Copies a path point so the builder never shares or modifies a point owned by the caller or by a built path.
     * A missing velocity is treated as at rest.
     * @param point the point to copy
     * @return a deep copy of the point
     */
    private static PathPoint copyOf(PathPoint point){
        PathPoint copy = new PathPoint();
        copy.time = point.time;
        copy.position = new Pos2D(point.position.x, point.position.y, point.position.angle);
        copy.velocity = point.velocity == null ? new Pos2D(0,0,0)
                : new Pos2D(point.velocity.x, point.velocity.y, point.velocity.angle);
        return copy;
    }
    /**
     * Replaces this builder's start with the given point, copying its pose, velocity and time.
     * Used when appending to a running sequence so the new path is continuous with the old one.
     * @param point the last point of the sequence being extended
     */
    void continueFrom(PathPoint point){
        PathPoint copy = copyOf(point);
        if (waypoints.isEmpty()) waypoints.add(new StartWaypoint(copy.position));
        else waypoints.set(0, new StartWaypoint(copy.position));
        previousCoordinate = copy.position;
        startingPoint = copy;
    }
    /**
     * Adds a start waypoint to the path using a defined coordinate.
     * Start waypoint must be the first waypoint in the path.
     * The robot will start at this waypoint and begin following the path from this point.
     * @param coordinate start position in inches and heading in degrees.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder start(Pos2D coordinate){
        return start(coordinate.x, coordinate.y, coordinate.angle);
    }
    /**
     * Adds an end waypoint to the path.
     * End waypoint must be the last waypoint in the path.
     * The robot will stop at this waypoint and end following the path from this point.
     * @param x end x coordinate in inches.
     * @param y end y coordinate in inches.
     * @param angle robot heading in degrees.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder end(double x, double y, double angle){
        end(x,y,angle,0.0);
        return this;
    }
    /**
     * Adds an end waypoint to the path.
     * End waypoint must be the last waypoint in the path.
     * The robot will stop at this waypoint and end following the path from this point.
     * @param x end x coordinate in inches.
     * @param y end y coordinate in inches.
     * @param angle robot heading in degrees.
     * @param endingSpeedRatio ending velocity ratio from 0 to 1
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder end(double x, double y, double angle, double endingSpeedRatio){
        waypoints.add(new EndWaypoint(x,y,angle));
        previousCoordinate = new Pos2D(x,y,angle);
        this.endingSpeedRatio = endingSpeedRatio;
        return this;
    }
    /**
     * Adds an end waypoint to the path using a defined coordinate.
     * End waypoint must be the last waypoint in the path.
     * The robot will stop at this waypoint and end following the path from this point.
     * @param coordinate end position in inches and heading in degrees.
     * @return The current SequenceBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder end(Pos2D coordinate){
        waypoints.add(new EndWaypoint(coordinate));
        previousCoordinate = coordinate;
        return this;
    }
    /**
     * Builds a sequence from the waypoints and actions added to this builder.
     * Path time starts at 0 seconds.
     * @return the compiled sequence
     * @throws NullPointerException if the waypoint list is null
     * @throws RuntimeException if the waypoint list is invalid, movement parameters are missing, or the sample rate is too small
     */
    public Sequence build(){
        return build(0);
    }
    /**
     * Builds a sequence from this builder's waypoints and actions, offset by a starting time.
     * @param time starting time in seconds
     * @return the compiled sequence
     * @throws NullPointerException if the waypoint list is null
     * @throws RuntimeException if the waypoint list is invalid, movement parameters are missing, or the sample rate is too small
     */
    public Sequence build(double time) {
        if (waypoints == null)
            throw new NullPointerException("Empty waypoints");
        if (waypoints.size() < 2)
            throw new RuntimeException("Cannot create path, must have at least two waypoints");
        if (waypoints.get(0).getType() != Waypoint.WaypointType.START)
            throw new RuntimeException("First Waypoint must be type START");
        if (waypoints.get(waypoints.size() - 1).getType() != Waypoint.WaypointType.END)
            throw new RuntimeException("Last Waypoint must be type END");
        if (movementParameters == null) throw new RuntimeException("Robot movement parameters must be defined");
        if (sampleRate <= 10e-9) throw new RuntimeException("SampleRate must be a positive, non negative number that is not lost in floating point rounding");

        // Work on a copy so the offset applies to this build only and the stored start is never modified
        PathPoint buildStart = copyOf(startingPoint);
        buildStart.time += time;

        ArrayList<Segment> segments = new ArrayList<Segment>();

        for (int i = 1; i < waypoints.size(); i++){
            Waypoint wp = waypoints.get(i);
            if (wp.getType() == Waypoint.WaypointType.END) {
                // Generate a spline segment that physically travels to the END position,
                // then a terminal WaitSegment so comeToStop decelerates the robot to zero.
                segments.add(new AngledSplineSegment(
                        getWaypointInRange(i - 2),
                        getWaypointInRange(i - 1),
                        wp,
                        wp));
                segments.add(new WaitSegment(wp.getCoordinate(), 0.4));
            } else {
                segments.add(createNewSegment(i));
            }
        }

        PathProcessor profileProcessor = new PathProcessor(movementParameters);
        ArrayList<Object> pathData = profileProcessor.processPath(segments.toArray(new Segment[0]), sampleRate, buildStart, endingSpeedRatio);
        @SuppressWarnings("unchecked")
        ArrayList<PathPoint> compiledPath = (ArrayList<PathPoint>) pathData.get(0);
        @SuppressWarnings("unchecked")
        ArrayList<Double> segmentTimeMarkers = (ArrayList<Double>) pathData.get(1);
        return new Sequence(compiledPath, actions.toArray(new SWEEPAction[0]), segmentTimeMarkers);
    }
    /**
     * Creates a new segment based on the waypoint type at the specified index.
     * This method is used internally by the build() method to create segments for the path.
     * @param waypointIndex The index of the waypoint in the waypoints array.
     * @return a segment representing the path between waypoints
     * @throws RuntimeException if the waypoint type has no segment implementation
     */
    private Segment createNewSegment(int waypointIndex){
        if (waypointIndex < 1 || waypoints.get(waypointIndex).getType() == Waypoint.WaypointType.START) throw new IllegalArgumentException("Cannot create segment on type START");
        Waypoint waypoint = getWaypointInRange(waypointIndex);
        switch (waypoint.getType()){
            case WAIT:
                return new WaitSegment(waypoint.getCoordinate(), waypoint.getDuration());
            case SPLINE:
                return new FollowSplineSegment(getWaypointInRange(waypointIndex-2),getWaypointInRange(waypointIndex-1),waypoint, getWaypointInRange(waypointIndex+1));
            case SPLINE_ANGLE:
                return new AngledSplineSegment(getWaypointInRange(waypointIndex-2),getWaypointInRange(waypointIndex-1),waypoint, getWaypointInRange(waypointIndex+1));
            case SPLINE_OFFSET:
                return new FollowSplineSegment(getWaypointInRange(waypointIndex-2),getWaypointInRange(waypointIndex-1),waypoint, getWaypointInRange(waypointIndex+1), waypoint.getHeadingOffset());
            case END:
                // END is handled directly in build() — a FollowSplineSegment is created there.
                throw new RuntimeException("END waypoints must be handled in build(), not createNewSegment()");
            case BREAK:
                return  new WaitSegment(waypoint.getCoordinate(), 1e-8);
        }
        throw new RuntimeException("Unable to create a new segment at waypoint IDX: " + waypointIndex);
    }
    /**
     * Gets the waypoint at the specified index, ensuring that the index is within the bounds of the waypoints array.
     * Responsible for handling path start and end cases where waypoints need to repeat the tangent waypoints
     * @param waypointIndex The index of the waypoint in the waypoints array.
     * @return The Waypoint object at the specified index.
     */
    private Waypoint getWaypointInRange(int waypointIndex){
        if (startingPoint == null) throw new IllegalArgumentException("Must have a defined starting Point");
        if (waypointIndex == -1) {
            return createWaypointFromStartingVelocity(startingPoint, waypoints.get(1));
        }
        waypointIndex = Math.max(waypointIndex,0);
        waypointIndex = Math.min(waypointIndex,waypoints.size()-1);
        return waypoints.get(waypointIndex);
    }
    /**
     * Creates the Catmull-Rom control point placed before the start so the path leaves the start
     * in the direction the robot is already moving.
     * <p>
     * Catmull-Rom sets the start tangent to 0.5 * (next - handle), so the handle is placed at
     * next - 2 * m * directionOfTravel, which gives a start tangent of length m (inches per unit of curve).
     * m is the distance the robot needs to brake from its current speed, v^2 / (2a), where a is the
     * max stable acceleration in the direction of travel. A faster start therefore gets a longer, straighter lead-in.
     * m is capped at the straight line distance to the next waypoint so close targets do not produce huge loops.
     * <p>
     * Known limitation (accepted for now): this only shapes the path geometry. The speed profile in PathProcessor
     * does not carry the start speed forward, so if the next waypoint is closer than the braking distance the
     * profile drops the speed faster than the max acceleration allows (the robot is asked to brake harder than it can).
     * The path is still finite and ends exactly on the requested point.
     * A zero start velocity gives m = 0, which places the handle on the next waypoint (the default Catmull-Rom start).
     * @param startingPoint the point the new path starts from, its velocity is in inches per second
     * @param nextPoint the first waypoint after the start
     * @return a waypoint to use as the control point before the start
     */
    private Waypoint createWaypointFromStartingVelocity(PathPoint startingPoint, Waypoint nextPoint) {
        double speed = startingPoint.velocity.getMagnitude();
        double directionRad = Math.atan2(startingPoint.velocity.y, startingPoint.velocity.x);
        double acceleration = movementParameters.getMaxStableAcceleration(Math.toDegrees(directionRad));
        double distanceToNext = Math.hypot(nextPoint.getX() - startingPoint.position.x, nextPoint.getY() - startingPoint.position.y);
        double brakingDistance = acceleration > 0 ? speed * speed / (2 * acceleration) : 0;
        double tangentLength = Math.min(brakingDistance, distanceToNext);
        Pos2D handlePos = new Pos2D(
                nextPoint.getX() - 2 * tangentLength * Math.cos(directionRad),
                nextPoint.getY() - 2 * tangentLength * Math.sin(directionRad),
                Math.toDegrees(directionRad));
        return new SplineAngleWaypoint(handlePos,1);
    }
    /**
     * Clips the given coordinate to be within the bounds of the field.
     * This method ensures that the robot does not attempt to drive outside the field boundaries.
     * @param coordinate The coordinate to be clipped.
     * @return The clipped coordinate, ensuring it is within the field boundaries.
     */
    private double clipCoordinateToField(double coordinate){
        // Clips to the field centre-to-edge distance only; robot size is not taken into account.
        coordinate = Math.min(coordinate, fieldLength/2);
        coordinate = Math.max(coordinate, -fieldLength/2);
        return coordinate;
    }
    /**
     * Clips the given speed to be within the range of 0 to 1.
     * This method ensures that the robot does not attempt to drive at an invalid speed.
     * @param speed The speed to be clipped.
     * @return The clipped speed, ensuring it is within the range of 0 to 1.
     */
    private double clipSpeedToRange(double speed){
        speed = Math.min(speed, 1);
        speed = Math.max(speed,0);
        return speed;
    }

}
