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
    /**
     * Length of standard FTC field, used to clip all passed in coordinates into the range of the field.
     * This is to prevent the robot from trying to drive outside the field boundaries.
     */
    private final double fieldLength = 144; // inches
    /**
     * The array of all waypoints that define the path.
     * Waypoints are added to this array using the various splineTo and linearTo methods.
     */
    private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    /**
     * The array of all actions that will be executed at specific points along the path.
     * Actions are added to this array using the addAction method.
     * Actions added need to extend the SWEEPActions class and be created through the specific robot class that extends SWEEPRobot
     * Actions will be executed in the order that they are added to the array,
     * and will be triggered when the robot is within a certain distance of the action's trigger.
     */
    private final ArrayList<SWEEPAction> actions = new ArrayList<SWEEPAction>();
    /**
     * The previous coordinate that was added to the path.
     * This is used to set the position of actions that are added to the path.
     * Allows the user to not specify a position for an action or wait they want to be at the last waypoint added to the path.
     */
    private Pos2D previousCoordinate;
    /**
     * The rate at which a path will be simulated and evaluated. Lower numbers means more accuracy with a tradeoff for slower processing times mainly during path rendering.
     * Default value is 0.01
     */
    private double sampleRate = 0.0005;
    private RobotMovementParameters movementParameters;
    private PathPoint startingPoint;
    private double endingSpeedRatio = 0.0;
    /**
     * Constructs a new PathBuilder object.
     * Initializes the waypoints and actions arrays, and sets the previous coordinate to (0,0,0).
     * @param movementParameters the definitions of the physical movement capabilities of the robot that will follow this path.
     */
    public SequenceBuilder(RobotMovementParameters movementParameters){
        previousCoordinate = new Pos2D(0,0,0);
        this.movementParameters = movementParameters;
    }
    public SequenceBuilder setSampleRate(double sampleRate){
        this.sampleRate = sampleRate;
        return this;
    }

    /**
     * Adds an action to the path.
     * If the action does not have a position set, it will be set to the previous coordinate.
     * Actions are executed in the order they are added to the path, and will be
     * triggered when the robot is within a certain distance of the action's trigger.
     * @param actionClass The action to be added to the path.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder addAction(SWEEPAction actionClass){
        if (actionClass == null) throw new NullPointerException("Action cannot be null");
        if (!actionClass.isPositionSet()) actionClass.setPosition(previousCoordinate);
        actions.add(actionClass);
        return this;
    }

    /**
     * Adds a spline waypoint to the path. Robot heading will follow the curve of the spline
     * @param x The x coordinate of the waypoint.
     * @param y The y coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineTo(double x, double y, double speed){
        speed = Math.min(speed, 1);
        speed = Math.max(speed,0);
        waypoints.add(new SplineWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),speed));
        previousCoordinate = new Pos2D(x,y,0);
        return this;
    }

    /**
     * Adds a spline waypoint to the path using a defined coordinate.
     * Robot heading will follow the curve of the spline.
     * @param definedCoordinate The coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineTo(Pos2D definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineWaypoint(definedCoordinate.x,definedCoordinate.y,speed));
        previousCoordinate = definedCoordinate;
        return this;
    }
    /**
     * Adds a spline waypoint to the path with a specified angle.
     * Robot heading will smoothly transition to the specified angle as it approaches the waypoint.
     * @param x The x coordinate of the waypoint.
     * @param y The y coordinate of the waypoint.
     * @param angle The angle at which the robot should be oriented at the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineToAngle(double x, double y, double angle, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineAngleWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),angle,speed));
        previousCoordinate = new Pos2D(x,y,0);
        return this;
    }
    /**
     * Adds a spline waypoint to the path with a specified angle using a defined coordinate.
     * Robot heading will smoothly transition to the specified angle as it approaches the waypoint.
     * @param definedCoordinate The coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder splineToAngle(Pos2D definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineAngleWaypoint(definedCoordinate.x,definedCoordinate.y,definedCoordinate.angle,speed));
        previousCoordinate = definedCoordinate;
        return this;
    }
    /**
     * Adds a break waypoint to the path.
     * break waypoints will force the robot to slow down and continue on the path, ignoring previous waypoint tangents and angles.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder addBreak(){
        waypoints.add(new BreakWaypoint(previousCoordinate));
        return this;
    }
    /**
     * Adds a wait waypoint to the path, similar to a break waypoint, but with a specified duration to wait at the previous waypoint before continuing on the path.
     * The robot will stop at the previous waypoint and wait for the specified duration before continuing on the path.
     * @param duration The duration in seconds to wait at the previous waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder waitAt(double duration){
        waypoints.add(new WaitWaypoint(previousCoordinate, duration));
        return this;
    }
    /**
     * Adds a start waypoint to the path, which must be the first waypoint in the path.
     * The robot will start at this waypoint and begin following the path from this point.
     * @param x The x coordinate of the start waypoint.
     * @param y The y coordinate of the start waypoint.
     * @param angle The angle at which the robot should be oriented at the start waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder start(double x, double y, double angle){
        PathPoint point = new PathPoint();
        point.time = 0;
        point.position = new Pos2D(x, y, angle);
        point.velocity = new Pos2D(0,0,0);
        start(point);
        return this;
    }
    public SequenceBuilder start(PathPoint startingPoint){
        waypoints.add(new StartWaypoint(startingPoint.position.x, startingPoint.position.y, startingPoint.position.angle));
        previousCoordinate = new Pos2D(startingPoint.position.x, startingPoint.position.y, startingPoint.position.angle);
        this.startingPoint = startingPoint;
        return this;
    }
    /**
     * Adds a start waypoint to the path using a defined coordinate.
     * Start waypoint must be the first waypoint in the path.
     * The robot will start at this waypoint and begin following the path from this point.
     * @param coordinate The coordinate of the start waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder start(Pos2D coordinate){
        waypoints.add(new StartWaypoint(coordinate));
        previousCoordinate = coordinate;
        return this;
    }
    /**
     * Adds an end waypoint to the path.
     * End waypoint must be the last waypoint in the path.
     * The robot will stop at this waypoint and end following the path from this point.
     * @param x The x coordinate of the end waypoint.
     * @param y The y coordinate of the end waypoint.
     * @param angle The angle at which the robot should be oriented at the end waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder end(double x, double y, double angle){
        end(x,y,angle,0.0);
        return this;
    }
    /**
     * Adds an end waypoint to the path.
     * End waypoint must be the last waypoint in the path.
     * The robot will stop at this waypoint and end following the path from this point.
     * @param x The x coordinate of the end waypoint.
     * @param y The y coordinate of the end waypoint.
     * @param angle The angle at which the robot should be oriented at the end waypoint.
     * @param endingSpeedRatio a value between 0.0 to 1.0
     * @return The current PathBuilder instance, allowing for method chaining.
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
     * @param coordinate The coordinate of the end waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public SequenceBuilder end(Pos2D coordinate){
        waypoints.add(new EndWaypoint(coordinate));
        previousCoordinate = coordinate;
        return this;
    }
    /**
     * Builds the path using the waypoints and actions added to the PathBuilder.
     * This method will create a Path object that can be used to follow the defined path.
     * Path time will start at 0 seconds.
     * @return A Path object that can be used to follow the defined path.
     */
    public Sequence build(){
        return build(0);
    }
    /**
     * Builds the path using the waypoints and actions added to the PathBuilder.
     * This method will create a Path object that can be used to follow the defined path.
     * @param time The starting time for the path, in seconds.
     * @return A Path object that can be used to follow the defined path.
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

//        if (startingPoint.velocity.getMagnitude() > 1e-5) {
//            createWaypointFromStartingVelocity(startingPoint, waypoints.get(1));
//        }

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
//                segments.add(new WaitSegment(wp.getCoordinate(), 0.1));
            } else {
                segments.add(createNewSegment(i));
            }
        }

        PathProcessor profileProcessor = new PathProcessor(movementParameters);
        return new Sequence(profileProcessor.processPath(segments.toArray(new Segment[0]), sampleRate, startingPoint, endingSpeedRatio),actions.toArray(new SWEEPAction[0]));
    }
    /**
     * Creates a new segment based on the waypoint type at the specified index.
     * This method is used internally by the build() method to create segments for the path.
     * @param waypointIndex The index of the waypoint in the waypoints array.
     * @return A Segment object that represents the path segment between waypoints.
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
            case END:
                // END is handled directly in build() — a FollowSplineSegment is created there.
                throw new RuntimeException("END waypoints must be handled in build(), not createNewSegment()");
            case BREAK:
                return  new WaitSegment(waypoint.getCoordinate(), 0.05);
            //TODO: Add more cases for new waypoint types as they are created
            //IDEA: Segment that forces the robot to always look at a specified coordinate on the field, with a linear and cubic spline version
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
    // Create a CatmullRom tangent waypoint for before the starting point to ensure that the generated path will start in the direction of the current momentum.
    private Waypoint createWaypointFromStartingVelocity(PathPoint startingPoint, Waypoint nextPoint) {
        double timestep = 12; //seconds TODO: Tune value relative to acceleration?
        double traveledDistance = startingPoint.velocity.getMagnitude() * timestep;
        double angle = Pos2D.getMovementDirectionRad(new Pos2D(0, 0, 0), startingPoint.velocity) + Math.PI; // backward through time
        Pos2D posAfterStep = new Pos2D(nextPoint.getX() - traveledDistance * Math.cos(angle), nextPoint.getY() - traveledDistance * Math.sin(angle), Math.toDegrees(angle-Math.PI));
//        Pos2D handlePos = new Pos2D((nextPoint.getX()+posAfterStep.x)/2,(nextPoint.getY()+posAfterStep.y)/2, startingPoint.position.angle);
        Pos2D handlePos = posAfterStep;
        System.out.println("Handle Pos: " + handlePos.x + ", " + handlePos.y);
        return new SplineAngleWaypoint(handlePos,1);
    }
    /**
     * Clips the given coordinate to be within the bounds of the field.
     * This method ensures that the robot does not attempt to drive outside the field boundaries.
     * @param coordinate The coordinate to be clipped.
     * @return The clipped coordinate, ensuring it is within the field boundaries.
     */
    private double clipCoordinateToField(double coordinate){
        //TODO: take the robots size and allow for the clipping to handle robot rotation. also allow user to specify a custom field size for clipping, in case two different alliances
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
