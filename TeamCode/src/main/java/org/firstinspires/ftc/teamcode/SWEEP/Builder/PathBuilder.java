package org.firstinspires.ftc.teamcode.SWEEP.Builder;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.Coordinate;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.SWEEPAction;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.Waypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.EndWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.BreakWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.LinearAngleWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.LinearWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segments.WaitSegment;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.SplineAngleWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.SplineWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.StartWaypoint;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.WaitWaypoint;

import java.util.ArrayList;

/**
 * PathBuilder is a class that allows for the creation of a path for a robot to follow.
 * It uses waypoints to define the path and actions to be executed at specific points along the path.
 * Class methods will build upon itself until the build() method is called, which will return a Path object that can be used to follow the defined path.
 */
public class PathBuilder {
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
    private Coordinate previousCoordinate;
    /**
     * Constructs a new PathBuilder object.
     * Initializes the waypoints and actions arrays, and sets the previous coordinate to (0,0,0).
     */
    public PathBuilder(){
        previousCoordinate = new Coordinate(0,0);
    }

    /**
     * Adds an action to the path.
     * If the action does not have a position set, it will be set to the previous coordinate.
     * Actions are executed in the order they are added to the path, and will be
     * triggered when the robot is within a certain distance of the action's trigger.
     * @param actionClass The action to be added to the path.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder addAction(SWEEPAction actionClass){
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
    public PathBuilder splineTo(double x, double y, double speed){
        speed = Math.min(speed, 1);
        speed = Math.max(speed,0);
        waypoints.add(new SplineWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),speed));
        previousCoordinate = new Coordinate(x,y,0);
        return this;
    }

    /**
     * Adds a spline waypoint to the path using a defined coordinate.
     * Robot heading will follow the curve of the spline.
     * @param definedCoordinate The coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder splineTo(Coordinate definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineWaypoint(definedCoordinate.getX(),definedCoordinate.getY(),speed));
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
    public PathBuilder splineToAngle(double x, double y, double angle, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineAngleWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),angle,speed));
        previousCoordinate = new Coordinate(x,y,0);
        return this;
    }
    /**
     * Adds a spline waypoint to the path with a specified angle using a defined coordinate.
     * Robot heading will smoothly transition to the specified angle as it approaches the waypoint.
     * @param definedCoordinate The coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder splineToAngle(Coordinate definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new SplineAngleWaypoint(definedCoordinate.getX(),definedCoordinate.getY(),definedCoordinate.getAngle(),speed));
        previousCoordinate = definedCoordinate;
        return this;
    }
    /**
     * Adds a linear waypoint to the path.
     * Robot heading will be fixed to the slope of the linear path between the previous waypoint and the new waypoint.
     * @param x The x coordinate of the waypoint.
     * @param y The y coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder linearTo(double x, double y, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new LinearWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),speed));
        previousCoordinate = new Coordinate(x,y,0);
        return this;
    }
    /**
     * Adds a linear waypoint to the path using a defined coordinate.
     * Robot heading will be fixed to the slope of the linear path between the previous waypoint and the new waypoint.
     * @param definedCoordinate The coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder linearTo(Coordinate definedCoordinate, double speed) {
        speed = clipSpeedToRange(speed);
        waypoints.add(new LinearWaypoint(definedCoordinate.getX(), definedCoordinate.getY(), speed));
        previousCoordinate = definedCoordinate;
        return this;
    }

    /**
     * Adds a break waypoint to the path.
     * break waypoints will force the robot to slow down and continue on the path, ignoring previous waypoint tangents and angles.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder addBreak(){
        waypoints.add(new BreakWaypoint(previousCoordinate));
        return this;
    }
    /**
     * Adds a linear waypoint with a specified angle to the path.
     * The robot heading will smoothly transition to the specified angle as it approaches the waypoint, while maintaining a linear path.
     * @param x The x coordinate of the waypoint.
     * @param y The y coordinate of the waypoint.
     * @param angle The angle at which the robot should be oriented at the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder linearToAngle(double x, double y, double angle, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new LinearAngleWaypoint(clipCoordinateToField(x),clipCoordinateToField(y),angle,speed));
        previousCoordinate = new Coordinate(x,y,angle);
        return this;
    }
    /**
     * Adds a linear waypoint with a specified angle using a defined coordinate.
     * The robot heading will smoothly transition to the specified angle as it approaches the waypoint, while maintaining a linear path.
     * @param definedCoordinate The coordinate of the waypoint.
     * @param speed The speed at which the robot should travel to the waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder linearToAngle(Coordinate definedCoordinate, double speed){
        speed = clipSpeedToRange(speed);
        waypoints.add(new LinearAngleWaypoint(definedCoordinate, speed));
        previousCoordinate = definedCoordinate;
        return this;
    }
    /**
     * Adds a wait waypoint to the path, similar to a break waypoint, but with a specified duration to wait at the previous waypoint before continuing on the path.
     * The robot will stop at the previous waypoint and wait for the specified duration before continuing on the path.
     * @param duration The duration in seconds to wait at the previous waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder waitAt(double duration){
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
    public PathBuilder start(double x, double y, double angle){
        waypoints.add(new StartWaypoint(x,y,angle));
        previousCoordinate = new Coordinate(x,y,angle);
        return this;
    }
    /**
     * Adds a start waypoint to the path using a defined coordinate.
     * Start waypoint must be the first waypoint in the path.
     * The robot will start at this waypoint and begin following the path from this point.
     * @param coordinate The coordinate of the start waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder start(Coordinate coordinate){
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
     * @param speed The speed at which the robot should travel to the end waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder end(double x, double y, double angle, double speed){
        waypoints.add(new EndWaypoint(x,y,angle,speed));
        previousCoordinate = new Coordinate(x,y,angle);
        return this;
    }
    /**
     * Adds an end waypoint to the path using a defined coordinate.
     * End waypoint must be the last waypoint in the path.
     * The robot will stop at this waypoint and end following the path from this point.
     * @param coordinate The coordinate of the end waypoint.
     * @param speed The speed at which the robot should travel to the end waypoint.
     * @return The current PathBuilder instance, allowing for method chaining.
     */
    public PathBuilder end(Coordinate coordinate, double speed){
        waypoints.add(new EndWaypoint(coordinate, speed));
        previousCoordinate = coordinate;
        return this;
    }
    /**
     * Builds the path using the waypoints and actions added to the PathBuilder.
     * This method will create a Path object that can be used to follow the defined path.
     * Path time will start at 0 seconds.
     * @return A Path object that can be used to follow the defined path.
     */
    public Path build(){
        return build(0);
    }
    /**
     * Builds the path using the waypoints and actions added to the PathBuilder.
     * This method will create a Path object that can be used to follow the defined path.
     * @param time The starting time for the path, in seconds.
     * @return A Path object that can be used to follow the defined path.
     */
    public Path build(double time) {
        if (waypoints == null)
            throw new NullPointerException("Empty waypoints");
        if (waypoints.size() < 2)
            throw new RuntimeException("Cannot create path, must have at least two waypoints");
        if (waypoints.get(0).getType() != Waypoint.WaypointType.START)
            throw new RuntimeException("First Waypoint must be type START");
        if (waypoints.get(waypoints.size() - 1).getType() != Waypoint.WaypointType.END)
            throw new RuntimeException("Last Waypoint must be type END");

        ArrayList<Segment> segments = new ArrayList<Segment>();

        for (int i = 1; i < waypoints.size(); i++){
            Segment segment = createNewSegment(i, time);
            time += segment.getTotalTime();
            segments.add(segment);
        }
        return new Path(segments.toArray(new Segment[0]), actions.toArray(new SWEEPAction[0]));
    }
    /**
     * Creates a new segment based on the waypoint type at the specified index.
     * This method is used internally by the build() method to create segments for the path.
     * @param waypointIndex The index of the waypoint in the waypoints array.
     * @param time The starting time for the segment, in seconds.
     * @return A Segment object that represents the path segment between waypoints.
     */
    private Segment createNewSegment(int waypointIndex,double time){
        if (waypointIndex < 1 || waypoints.get(waypointIndex).getType() == Waypoint.WaypointType.START) throw new IllegalArgumentException("Cannot create segment on type START");
        Waypoint waypoint = getWaypointInRange(waypointIndex);
        switch (waypoint.getType()){
            case END:
                //TODO create end segment class and generation algorithm
                break;
            case WAIT:
                //TODO: refine the wait segment to have the robot come to a smooth stop
                return new WaitSegment(waypoint.getCoordinate(),time, waypoint.getDuration());
            case BREAK:
                //TODO create break segment class and generation algorithm
                break;
            case SPLINE:
                //TODO create spline segment class and generation algorithm
                break;
            case SPLINE_ANGLE:
                //TODO create spline angle segment class and generation algorithm
                break;
            case LINEAR:
                //TODO create linear segment class and generation algorithm
                break;
            default: // Also case for LinearAngle
                //TODO create linear angle segment class and generation algorithm
                break;
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
        waypointIndex = Math.max(waypointIndex,0);
        waypointIndex = Math.min(waypointIndex,waypoints.size()-1);
        return waypoints.get(waypointIndex);
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

