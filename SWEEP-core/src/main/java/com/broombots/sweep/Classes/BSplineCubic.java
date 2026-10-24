package com.broombots.sweep.Classes;

import org.ejml.simple.SimpleMatrix;

/** Evaluates a scalar uniform cubic B-spline from four control values. */
public class BSplineCubic {
    /** Four scalar control values used by the cubic basis, in the caller's units. */
    private final SimpleMatrix controlPoints;

    /** Creates a cubic B-spline from four scalar control values.
     * @param point1 first control value
     * @param point2 second control value
     * @param point3 third control value
     * @param point4 fourth control value
     */
    public BSplineCubic(double point1, double point2, double point3, double point4){
        controlPoints = new SimpleMatrix(new double[][]{
            {point1, point2, point3, point4}
        });
    }
    /** Returns the cubic B-spline basis weight for one control point. */
    private double basis(int i, double t) {
        switch (i) {
            case 0:
                return ((-t * t * t + 3 * t * t - 3 * t + 1) / 6.0);
            case 1:
                return ((3 * t * t * t - 6 * t * t + 4) / 6.0);
            case 2:
                return ((-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0);
            case 3:
                return ((t * t * t) / 6.0);
            default:
                return 0.0;
        }
    }
    /** Evaluates the spline at a normalized parameter value.
     * @param t spline parameter, conventionally in the range from 0 to 1
     * @return interpolated scalar in the same units as the control values
     */
    public double evaluate(double t){
        return basis(0, t) * controlPoints.get(0) +
                basis(1,t) * controlPoints.get(1) +
                basis(2,t) * controlPoints.get(2) +
                basis(3,t) * controlPoints.get(3);
    }
}
