package org.firstinspires.ftc.teamcode.SWEEP.Classes;

public interface RobotMovementParameters {
    // Max velocity parameters
    public double getMaxVelocity();

    // Max Acceleration parameters
    public double getMaxStableAcceleration(double direction);
    public double getMaxStableAngularAcceleration();

    // Min Acceleration parameters
    public double getForwardMinAcceleration();
    public double getBackwardMinAcceleration();
    public double getRightMinAcceleration();
    public double getLeftMinAcceleration();
    public double getClockwiseMinAcceleration();
    public double getCounterClockwiseMinAcceleration();

    // Weight Imbalance Parameters

    //

}
