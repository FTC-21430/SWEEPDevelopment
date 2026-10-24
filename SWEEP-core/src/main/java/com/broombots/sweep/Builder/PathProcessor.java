package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.LerpFunction;
import com.broombots.sweep.Classes.LerpFunctionBuilder;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Splines.Segment;
import com.broombots.sweep.Splines.Segments.WaitSegment;

import java.util.ArrayList;

/**
 * Compiles spline segments into a time-parameterized movement profile.
 * <p>
 * Processing has four stages:
 * <ol>
 *     <li>Split the input at waits so a stationary interval is not treated as a moving spline.</li>
 *     <li>Sample each movement segment by its dimensionless spline parameter {@code t}.</li>
 *     <li>Assign a speed to each sample. The forward pass limits acceleration from the start; the backward
 *     pass limits speed where the robot must brake for the end of a run.</li>
 *     <li>Convert distance and average adjacent speeds into elapsed time.</li>
 * </ol>
 * <p>
 * Translational positions and distances use inches, translational velocity uses inches per second, and
 * timestamps use seconds. Headings use degrees except for the local curvature estimate, whose heading
 * change is in radians so curvature is expressed in radians per inch.
 * <p>
 * This class stores sampled path geometry separately from its velocity profile: spline segments provide
 * positions, then the two passes fill translational velocities, then time is integrated from those values.
 * Angular velocity is currently calculated in the passes but intentionally is not written into the output
 * {@link PathPoint#velocity}; output angular velocity therefore remains zero here.
 */
public class PathProcessor {
    /** Robot motion limits used to create the velocity profile; distances are in inches and speeds in inches per second. */
    private final RobotMovementParameters movementParameters;
    /** Last rendered position; coordinates are in inches and heading is in degrees. */
    private Pos2D lastRenderedPosition = null;
    /**
     * Hand-tuned lookup from local curvature in radians per inch to a speed scale.
     * A value of 1 preserves the movement-parameter speed limit; smaller values reduce speed in tighter turns.
     */
    private final LerpFunction regression;
    /** Per-sample speed scales from the corresponding segments. */
    private final ArrayList<Double> SPEED_RATIOS = new ArrayList<>();

    /**
     * Creates a processor using the robot's motion limits.
     * @param movementParameters robot motion limits, including velocity and acceleration
     */
    public PathProcessor(RobotMovementParameters movementParameters) {
        this.movementParameters = movementParameters;

        // These shared breakpoints trade speed for tighter curvature; they are not yet robot-specific tuning values.
        regression = new LerpFunctionBuilder()
                .addPoint(0,1)
                .addPoint(0.03, 0.98)
                .addPoint(0.04,0.8)
                .addPoint(0.35, 0.25)
                .build();
    }
    /**
     * Compiles segments into sampled positions and their segment start times.
     * @param segments spline and wait segments to compile
     * @param sampleRate spline parameter step between samples, from 0 to 1
     * @param startingPoint initial pose, velocity, and time; position is in inches, heading in degrees, velocity in inches per second, and time in seconds
     * @param endingSpeedRatio desired ending speed as a ratio from 0 to 1
     * @return a two-element list: {@code ArrayList<PathPoint> path} followed by {@code ArrayList<Double> segmentTimeMarkers} in seconds
     */
    public ArrayList<Object> processPath(Segment[] segments, double sampleRate, PathPoint startingPoint, double endingSpeedRatio) {
        ArrayList<Double> segmentTimeMarkers = new ArrayList<>();
        ArrayList<PathPoint> path = new ArrayList<>();
        ArrayList<ArrayList<Segment>> brokenSegments = breakUpComplexRoute(segments);
        for (int i = 0; i < brokenSegments.size(); i++){
            ArrayList<Segment> segmentList = brokenSegments.get(i);
            if (segmentList.get(0) instanceof WaitSegment){
                // A wait is processed against elapsed sequence time and holds the previous rendered pose.
                path.addAll(processWaitSegments(segmentList.toArray(new Segment[0]), sampleRate, path.isEmpty() ? startingPoint.time : path.get(path.size()-1).time, segmentTimeMarkers));
            }else{
                // Any run followed by another run must stop at its boundary. Only the final run may keep the
                // caller's requested ending speed; this prevents carrying motion through a wait.
                double speedRate = i < brokenSegments.size()-1? 0 : endingSpeedRatio;
                PathPoint prevPoint = i > 0? path.get(path.size()-1) : startingPoint;
                path.addAll(processMovementSegments(segmentList.toArray(new Segment[0]), sampleRate, prevPoint, speedRate, segmentTimeMarkers));
            }
        }
        // Segments calculate their own headings; unwrap them after concatenation so the boundary between
        // independently generated segments cannot create an artificial near-360-degree rotation.
        makeHeadingContinuous(path);
        ArrayList<Object> result = new ArrayList<>();
        result.add(path);
        result.add(segmentTimeMarkers);
        return result;
    }
    /** Adjusts path headings so adjacent samples do not introduce full-turn discontinuities. */
    private void makeHeadingContinuous(ArrayList<PathPoint> path){
        for (int i = 1; i < path.size(); i++){
            double previous = path.get(i-1).position.angle;
            Pos2D position = path.get(i).position;
            if (Double.isNaN(previous) || Double.isNaN(position.angle)) continue;
            // Add the shortest signed difference to the previous unwrapped heading. For example, 350° -> 10°
            // becomes 350° -> 370°, rather than asking the robot to rotate backward by 340°.
            position.angle = previous + wrapDegrees(position.angle - previous);
        }
    }
    /**
     * Splits the route into contiguous movement groups with each wait kept as a standalone group.
     * The caller uses these boundaries to ensure each movement group reaches zero speed before a wait.
     */
    private ArrayList<ArrayList<Segment>> breakUpComplexRoute(Segment[] segments){
        ArrayList<ArrayList<Segment>> result = new ArrayList<>();
        ArrayList<Segment> currentSegmentList = new ArrayList<>();
        for (Segment segment : segments){
            if (segment instanceof WaitSegment){
                if (!currentSegmentList.isEmpty()) result.add(currentSegmentList);
                ArrayList<Segment> waitSegmentList = new ArrayList<>();
                waitSegmentList.add(segment);
                result.add(waitSegmentList);
                currentSegmentList = new ArrayList<>();
            }else{
                currentSegmentList.add(segment);
            }
        }
        if (!currentSegmentList.isEmpty()) result.add(currentSegmentList);
        return result;
    }
    /**
     * Creates zero-velocity samples for a group of wait segments.
     * Each wait segment contributes its start-time marker and samples at {@code sampleRate} intervals.
     * The duration is in seconds; unlike movement sampling, the loop variable here is elapsed time.
     */
    private ArrayList<PathPoint> processWaitSegments(Segment[] segments, double sampleRate, double startTime, ArrayList<Double> segmentTimeMarkers){
        ArrayList<PathPoint> path = new ArrayList<>();
        double lastTime = startTime;
        for (Segment segment : segments){
            segmentTimeMarkers.add(lastTime);
            WaitSegment waitSegment = (WaitSegment) segment;
            if (waitSegment.getDuration() <= 1e-7) {
                // Keep a zero-duration wait's pose in the path even though it adds no meaningful elapsed time.
                Pos2D pos2D = waitSegment.getPosition(0);
                PathPoint point = new PathPoint();
                point.position = pos2D;
                point.velocity = new Pos2D(0,0,0);
                point.time = lastTime;
                path.add(point);
                continue;
            }
            for (double t = lastTime; t < waitSegment.getDuration()+lastTime; t += sampleRate){
                Pos2D pos2D = waitSegment.getPosition(t);
                if (Double.isNaN(pos2D.angle)){
                    // A wait without an explicit heading inherits the last known robot heading.
                    pos2D.angle = lastRenderedPosition.angle;
                }
                PathPoint point = new PathPoint();
                point.position = pos2D;
                point.velocity = new Pos2D(0,0,0);
                point.time = t;
                path.add(point);
            }
            lastTime = path.get(path.size()-1).time;
        }
        lastRenderedPosition = path.get(path.size()-1).position;
        return path;
    }
    /**
     * Samples movement geometry, generates the velocity profile, and assigns elapsed time.
     * Segment positions are sampled by dimensionless spline parameter {@code t} from 0 to 1;
     * {@code sampleRate} is therefore not a time step. Segment boundaries are recorded as path indices
     * first, then translated into times after the velocity profile is known.
     */
    private ArrayList<PathPoint> processMovementSegments(Segment[] segments, double sampleRate, PathPoint startingPoint, double endingSpeedRatio, ArrayList<Double> segmentTimeMarkers){
        ArrayList<PathPoint> path = new ArrayList<>();
        ArrayList<Integer> waypointIndexInPath = new ArrayList<>();

        if (Double.isNaN(startingPoint.position.angle)){
            // The first movement after an angle-unspecified wait continues from the last rendered heading.
            startingPoint.position = lastRenderedPosition;
        }

        path.add(startingPoint);
        // Keep a speed ratio for each point added to the path. The start point uses the first segment's
        // ratio; every sampled point uses its own segment's ratio. These ratios scale the local speed cap.
        SPEED_RATIOS.add(segments[0].getSpeedRate());
        for (Segment segment : segments) {
            waypointIndexInPath.add(path.size());
            for (double t = 0.0 + 1e-7; t < 1.0; t += sampleRate) {
                Pos2D pos2D = segment.getPosition(t);
                PathPoint point = new PathPoint();
                point.position = pos2D;
                path.add(point);
                SPEED_RATIOS.add(segment.getSpeedRate());
            }
        }
        waypointIndexInPath.add(path.size());
        PathPoint endPoint = new PathPoint();
        endPoint.position = segments[segments.length-1].getPosition(1);

        // Seed the final sample with the requested terminal speed along the last segment's travel direction.
        // The backward pass then propagates the braking constraint to earlier samples.
        double endAngle = Pos2D.getMovementDirectionRad(path.get(path.size() - 1).position, endPoint.position);
        double terminalSpeed = movementParameters.getMaxVelocity(Math.toRadians(endAngle)) * Math.max(0.0, Math.min(1.0, endingSpeedRatio));
        endPoint.velocity = new Pos2D(
                terminalSpeed * Math.cos(endAngle),
                terminalSpeed * Math.sin(endAngle),
                0
        );
        path.add(endPoint);

        path = generatePathPoints(path);

        // Integrate time between samples using distance / average speed (trapezoidal approximation).
        // This is more accurate than using only the previous sample's speed when speed changes along the path.
        double renderTime = startingPoint.time;
        segmentTimeMarkers.add(renderTime);

        for (int i = 1; i < path.size(); i++){
            PathPoint lastPoint = path.get(i-1);
            PathPoint currentPoint = path.get(i);

            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double avgVelocity = (lastPoint.velocity.getMagnitude()+currentPoint.velocity.getMagnitude())/2.0;
            double dt;
            if (avgVelocity > 1e-5) {
                dt = distance / avgVelocity;
            }else{
                // Avoid division by zero for duplicate or stationary samples; keep timestamps strictly increasing.
                dt = 1e-5;
            }

            renderTime += dt;
            if (waypointIndexInPath.contains(i)){
                segmentTimeMarkers.add(renderTime);
            }


            path.get(i).time = renderTime;
        }

        SPEED_RATIOS.clear();
        lastRenderedPosition = path.get(path.size()-1).position;
        return path;
    }

    /**
     * Assigns translational velocities to an already sampled geometric path.
     * <p>
     * The forward pass enforces reachable acceleration from the starting velocity and caps speed by
     * curvature and segment speed ratios. The backward pass works from the terminal velocity and enforces
     * the same acceleration limit while braking. Together these passes produce a profile that can both
     * accelerate from the start and arrive at the requested ending speed.
     */
    private ArrayList<PathPoint> generatePathPoints(ArrayList<PathPoint> path) {
        // Forward pass: given the speed at sample i-1, find the greatest speed reachable at sample i,
        // then clamp it to the local speed goal (max speed, curvature reduction, and segment ratio).
        for (int i = 1; i < path.size()-2; i++){

            PathPoint lastPoint = path.get(i-1);
            PathPoint currentPoint = path.get(i);
            PathPoint nextPoint = null;
            if (i < path.size()-2){
                nextPoint = path.get(i+1);
            }
            double d1 = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);

            double adjustedSpeed = calculateSpeedGoal(lastPoint,currentPoint,nextPoint, i);
            // v^2 = u^2 + 2*a*s gives the maximum speed reachable after traveling d1 inches
            // with acceleration a (inches/s^2), starting at lastVelocityScalar (inches/s).
            double lastVelocityScalar = Math.hypot(lastPoint.velocity.x, lastPoint.velocity.y);
            double momentAcceleration = movementParameters.getMaxStableAcceleration(Math.toDegrees(Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position)));
            double newVelocityScalar = calculateNewVelocity(d1, momentAcceleration, lastVelocityScalar);
            if (newVelocityScalar > adjustedSpeed) newVelocityScalar = adjustedSpeed;
            double angleRad = Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position);
            double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);
            // Angular speed is profiled with the analogous rotational equation, using degrees and
            // degrees/second. It is currently diagnostic/incomplete: translational PathPoint velocity is
            // intentionally written with angular component 0 below.
            double angularDifference = wrapDegrees(currentPoint.position.angle - lastPoint.position.angle);
            double angularSign = Math.signum(angularDifference);
            double angularVelocity = calculateNewVelocity(Math.abs(angularDifference), movementParameters.getMaxStableAngularAcceleration(newVelocityScalar, angularSign > 0), Math.abs(lastPoint.velocity.angle));
            angularVelocity *= angularSign;
            if (Math.abs(angularVelocity) > movementParameters.getMaxAngleVelocity()) angularVelocity = movementParameters.getMaxAngleVelocity() * angularSign;
            Pos2D velocity = new Pos2D(xVelocity,yVelocity,0);
            path.get(i).velocity = velocity;


        }

        // Backward pass: traverse from the end toward the start. Applying the same equation in reverse
        // computes the greatest speed at each earlier sample that can still brake to the next sample's speed.
        for (int i = path.size()-2; i > 1; i--){
            PathPoint lastPoint = path.get(i+1);
            PathPoint currentPoint = path.get(i);

            if (currentPoint.velocity == null) currentPoint.velocity = new Pos2D(0,0,0);
            double distance = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
            double lastVelocityScalar = Math.hypot(lastPoint.velocity.x, lastPoint.velocity.y);

            double forwardPassVelocityScalar = Math.hypot(currentPoint.velocity.x, currentPoint.velocity.y);
            // At this point the later sample is the "starting" speed and distance is measured back to
            // the current sample. If the forward pass already assigned an even lower speed, preserve it.
            double newVelocityScalar = calculateNewVelocity(distance, movementParameters.getMaxStableAcceleration(Math.toDegrees(Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position))), lastVelocityScalar);
            if (forwardPassVelocityScalar < newVelocityScalar && i != path.size()-2) continue; // respect the forward pass output
            // The vector direction must still point forward along the route, even though the iteration
            // order is backward.
            double angleRad = Pos2D.getMovementDirectionRad(lastPoint.position, currentPoint.position);
            double xVelocity = newVelocityScalar * Math.cos(angleRad);
            double yVelocity = newVelocityScalar * Math.sin(angleRad);

            // Angular velocity profiling is computed but not yet applied to output velocity.
            double angularDifference = wrapDegrees(currentPoint.position.angle - lastPoint.position.angle);
            double angularSign = Math.signum(angularDifference);
            double angularVelocity = calculateNewVelocity(Math.abs(angularDifference), movementParameters.getMaxStableAngularAcceleration(Math.hypot(currentPoint.velocity.x, currentPoint.velocity.y), angularSign > 0), Math.abs(lastPoint.velocity.angle));
            angularVelocity *= angularSign;
            if (Math.abs(angularVelocity) > movementParameters.getMaxAngleVelocity()) angularVelocity = movementParameters.getMaxAngleVelocity() * angularSign;

            Pos2D velocity = new Pos2D(xVelocity,yVelocity,0);
            currentPoint.velocity = velocity;
        }
        return path;
    }

    /** Calculates the next velocity under the configured acceleration limit. */
    private double calculateNewVelocity(double dTraveled, double acceleration, double lastVelocity){
        // Constant-acceleration kinematics: v^2 = u^2 + 2*a*s. Units must match:
        // acceleration in distance/s^2, distance in the same distance unit, and velocity in distance/s.
        return Math.sqrt(Math.pow(lastVelocity, 2) + 2 * (acceleration * dTraveled));
    }

    /**
     * Computes the local translational speed ceiling for one sampled point.
     * Curvature is estimated from the absolute change in travel direction divided by average distance
     * between the neighboring samples, yielding radians per inch. The configured lookup maps that value
     * to a speed multiplier, then the segment's own speed ratio scales the robot's directional speed limit.
     */
    private double calculateSpeedGoal(PathPoint lastPoint, PathPoint currentPoint, PathPoint nextPoint, int idx){
        // Estimate curvature from the two adjacent travel vectors. Straight travel has zero heading change;
        // sharp bends have a larger change over a shorter distance and therefore a larger curvature value.
        double d1 = Pos2D.getDistanceBetweenCoordinates(lastPoint.position, currentPoint.position);
        double d2;
        double angle1 = Pos2D.getMovementDirectionRad(lastPoint.position,currentPoint.position);
        double angle2;
        if (nextPoint != null){
            d2 = Pos2D.getDistanceBetweenCoordinates(currentPoint.position, nextPoint.position);
            angle2 = Pos2D.getMovementDirectionRad(currentPoint.position,nextPoint.position);
        }else {
            d2 = d1;
            angle2 = angle1;
        }
        double averageTravelDistance = Math.abs((d1+d2)/2.0);
        double changeInAngleRAD = Math.abs(wrapRadians(angle2 - angle1));
        double deltaRadPerInch;
        if (changeInAngleRAD < 1e-8 || averageTravelDistance < 1e-3){
            // Treat tiny numerical differences and nearly duplicate samples as straight to avoid noisy or
            // unbounded curvature values from division by an almost-zero distance.
            deltaRadPerInch = 0;
        }
        else{
            deltaRadPerInch = changeInAngleRAD/averageTravelDistance;
        }
        // Movement parameters expect direction in degrees; regression input remains radians per inch.
        return movementParameters.getMaxVelocity(Math.toDegrees(Pos2D.getMovementDirectionRad(currentPoint.position, lastPoint.position))) * regression.getValue(deltaRadPerInch) * SPEED_RATIOS.get(idx);
    }
    /** Normalizes an angle difference in radians to the shortest signed equivalent in [-pi, pi]. */
    private double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
    /** Normalizes an angle difference in degrees to the shortest signed equivalent in [-180, 180]. */
    private double wrapDegrees(double angle){
        double resultingAngle = angle;
        while (resultingAngle > 180) resultingAngle -= 360;
        while (resultingAngle < -180) resultingAngle += 360;
        return resultingAngle;
    }

}
