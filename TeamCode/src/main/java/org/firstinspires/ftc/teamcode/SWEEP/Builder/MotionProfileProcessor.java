package org.firstinspires.ftc.teamcode.SWEEP.Builder;

import org.firstinspires.ftc.teamcode.SWEEP.Classes.CatmullRomCubic;
import org.firstinspires.ftc.teamcode.SWEEP.Classes.RobotMovementParameters;
import org.firstinspires.ftc.teamcode.SWEEP.Splines.Segment;

public class MotionProfileProcessor {
    private RobotMovementParameters movementParameters;
    public MotionProfileProcessor(RobotMovementParameters movementParameters) {
        // Initialize the motion profile processor with the given movement parameters
        this.movementParameters = movementParameters;
    }
    public void updateMovementParameters(RobotMovementParameters movementParameters) {
        // Update the movement parameters for the motion profile processor
        this.movementParameters = movementParameters;
    }
    public MovementMap processPath(Segment[] segments, double sampleRate) {
              MovementMap movementMap = new MovementMap(sampleRate);
             return movementMap;
    }

    private double[] findMaxCurvePoints(Segment segment){
        return new double[]{0, 1}; // TODO: implement
    }
    private void compileVelocityProfile(Segment segment){
        double[] maxCurvePoints = findMaxCurvePoints(segment);
        //Algorithm steps
        // 2. Find the max curve points of the segment
        // 3. Use the curve of those points to determine the max robot velocity at that curve ( and therefore the slowest theoretically in the spline the robot could move at)
        // 4. Simulate at a sample rate of distance, the fastest the robot would be able to accelerate from each of those distances
        // 5. Compute full velocity profile by comparing each simulation and taking the lowest velocity at each point
        // 6. iterate through the velocity profile to get a full movementmap that represents the segment
        // 7. return that MovementMap which is the velocity profile.
    }

