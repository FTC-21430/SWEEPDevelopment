package com.broombots.sweep.Movement;

import com.broombots.sweep.Classes.SWEEPTimer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Computes a bounded PID power output for a scalar target.
 * Target and error use the caller's units, while output power is in the range -1 to 1.
 */
@Disabled
public class SWEEPPIDController {
  /** Proportional gain for the target's unit domain. */
  private double pConstant;
  /** Integral gain for the target's unit domain and seconds. */
  private double iConstant;
  /** Derivative gain for the target's unit domain per second. */
  private double dConstant;
  /** Accumulated error over elapsed seconds. */
  private double iSum=0;
  /** Integral gain restored when integral mode is enabled. */
  private final double iNormal;
  /** Bounded controller output, in motor power from -1 to 1. */
  private double power;
  /** Proportional contribution to output power, retained for diagnostics. */
  private double powerProportional;
  /** Integral contribution to output power, retained for diagnostics. */
  private double powerIntegral;
  /** Derivative contribution to output power, retained for diagnostics. */
  private double powerDerivative;
  /** Target value in the caller's unit domain. */
  private double target;
  /** Previous update time in seconds and previous error in target units. */
  private double lastTime, lastError;
  /** Timer used to calculate update intervals, in seconds. */
  private final SWEEPTimer runtime;
  /** Whether errors are wrapped as angles in degrees. */
  private boolean shouldWrap = true;
  /** Stored wrap bounds in target units; currently unused by {@link #update(double)}. */
  private double minWrap = 0.0, maxWrap = 0.0;

  /**
   * Creates a controller that wraps errors as angles in degrees.
   * @param pConstant proportional gain
   * @param iConstant integral gain
   * @param dConstant derivative gain
   * @param runtime timer supplying elapsed time in seconds
   */
  public SWEEPPIDController(double pConstant, double iConstant, double dConstant, SWEEPTimer runtime) {
    this.pConstant = pConstant;
    this.dConstant = dConstant;
    this.iConstant = iConstant;
    this.runtime = runtime;
    shouldWrap = true;

    iNormal = iConstant;

    // to ensure lastTime is correct for the first iteration of update.
    lastTime = runtime.getSeconds();
  }

  /**
   * Creates a controller with optional angular error wrapping.
   * @param pConstant proportional gain
   * @param iConstant integral gain
   * @param dConstant derivative gain
   * @param runtime timer supplying elapsed time in seconds
   * @param shouldWrap whether to wrap error as an angle in degrees
   */
  public SWEEPPIDController(double pConstant, double iConstant, double dConstant, SWEEPTimer runtime, boolean shouldWrap) {
    this.pConstant = pConstant;
    this.dConstant = dConstant;
    this.iConstant = iConstant;
    this.runtime = runtime;
    this.shouldWrap = shouldWrap;

    iNormal = iConstant;

    // to ensure lastTime is correct for the first iteration of update.
    lastTime = runtime.getSeconds();
  }

  /**
   * Replaces the controller gains.
   * @param pConstant proportional gain
   * @param iConstant integral gain
   * @param dConstant derivative gain
   */
  public void updatePIDConstants(double pConstant, double iConstant, double dConstant){
    this.pConstant = pConstant;
    this.dConstant = dConstant;
    this.iConstant = iConstant;
  }

  /**
   * Updates the bounded output power from the current measurement.
   * @param currentPosition current value in the target's unit domain
   */
  public void update(double currentPosition) {
     double deltaTime = (runtime.getSeconds()-lastTime);
    // the error of how far you are from where you want to be
    // added wrap to the error equation
    double error = currentPosition - target;
    if (shouldWrap) {
      error = Utilities.wrap(error);
    }
    iSum += error *deltaTime;

    double derivative;
    if (deltaTime > 1e-7) {

      // The derivative factor scales down the Proportional factor so that we don't over shoot our target.
      derivative = (error - lastError) / deltaTime;
    }else{
      derivative = 0;
    }
    
    // the proportional factor affected by the derivative factor to get our output
    power = -error * pConstant +(-iSum * iConstant) + (-dConstant * derivative);

    if (Math.abs(power) >1){
      iSum -= error * deltaTime;
      power = -error * pConstant  + (-dConstant * derivative);
    }

    powerProportional= -error * pConstant;
    powerIntegral= (-iSum * iConstant);
    powerDerivative= (-dConstant * derivative);
    // to ensure that power is always within our range of -1 - 1
    // if power is - 5 then it is divided by 5 to give us -1
    if (power > 1 || power < -1) power /= Math.abs(power);
    
    // updates the last variables that we need for the next iteration of the loop.
    lastTime+= deltaTime;
    lastError = error;
  }
  
  /**
   * Sets the target value.
   * @param target target in the caller's unit domain
   */
  public void setTarget(double target) {
    this.target = target;
  }
  
  /**
   * Gets the target value.
   * @return target in the caller's unit domain
   */
  public double getTarget() {
    return target;
  }
  
  /**
   * Stores wrap bounds; these values are currently unused by {@link #update(double)}, which wraps to +/-180 degrees.
   * @param min minimum bound in degrees
   * @param max maximum bound in degrees
   */
  public void setWrap(double min, double max){
    minWrap = min;
    maxWrap = max;
  }

  /**
   * Enables or disables integral contribution.
   * @param mode {@code true} to restore the configured integral gain, or {@code false} to disable it
   */
  public void setIntegralMode(boolean mode){
    if(mode){
      if (iConstant == 0){
        iSum = 0;
      }
      iConstant = iNormal;
    }else{
      iConstant = 0;
    }
  }

  /**
   * Gets the most recently calculated output power.
   * @return output power from -1 to 1
   */
  public double getPower() {
    return power;
  }
  /**
   * Gets the proportional gain.
   * @return proportional gain
   */
  public double getPConstant(){
    return pConstant;
  }
  /**
   * Gets the integral gain.
   * @return integral gain
   */
  public double getIConstant(){
    return iConstant;
  }
  /**
   * Gets the derivative gain.
   * @return derivative gain
   */
  public double getDConstant(){
    return dConstant;
  }
}
