package org.firstinspires.ftc.teamcode.SWEEP.Classes;

public interface RobotMovementParameters {
    // Max velocity parameters
    public double getForwardMaxVelocity();
    public double getBackwardMaxVelocity();
    public double getRightMaxVelocity();
    public double getLeftMaxVelocity();
    public double getClockwiseMaxVelocity();
    public double getCounterClockwiseMaxVelocity();

    // Max Acceleration parameters
    public double getForwardMaxAcceleration();
    public double getBackwardMaxAcceleration();
    public double getRightMaxAcceleration();
    public double getLeftMaxAcceleration();
    public double getClockwiseMaxAcceleration();
    public double getCounterClockwiseMaxAcceleration();

    // Min Acceleration parameters
    public double getForwardMinAcceleration();
    public double getBackwardMinAcceleration();
    public double getRightMinAcceleration();
    public double getLeftMinAcceleration();
    public double getClockwiseMinAcceleration();
    public double getCounterClockwiseMinAcceleration();

    // Max Jerk Parameters (change in acceleration)
    public double getForwardMaxJerk();
    public double getBackwardMaxJerk();
    public double getRightMaxJerk();
    public double getLeftMaxJerk();
    public double getClockwiseMaxJerk();
    public double getCounterClockwiseMaxJerk();

    // Weight Imbalance Parameters

}
