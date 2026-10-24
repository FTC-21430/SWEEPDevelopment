package com.broombots.sweep.Classes;

import org.ejml.simple.SimpleMatrix;

/** Evaluates a scalar uniform Catmull-Rom cubic from four control values. */
public class CatmullRomCubic {
    /** Polynomial coefficients [a, b, c, d] for the scalar curve. */
    private final SimpleMatrix coeffs;

    /** Creates a uniform Catmull-Rom cubic from four scalar samples.
     * @param point1 sample before the segment start, in the caller's units
     * @param point2 value at the segment start
     * @param point3 value at the segment end
     * @param point4 sample after the segment end
     */
    public CatmullRomCubic(double point1, double point2, double point3, double point4) {
        coeffs = new SimpleMatrix(catmullRomCoeffs(point1, point2, point3, point4));
    }
    /** Evaluates the curve at a normalized parameter value.
     * @param t normalized curve parameter, conventionally from 0 to 1
     * @return scalar value in the same units as the input samples
     */
    public double evaluate(double t) {
        double a = coeffs.get(0, 0);
        double b = coeffs.get(0, 1);
        double c = coeffs.get(0, 2);
        double d = coeffs.get(0, 3);
        return a * Math.pow(t,3) + b * Math.pow(t,2) + c * t + d;
    }
    /** Computes the derivative of the curve with respect to its normalized parameter.
     * @param t normalized curve parameter, conventionally from 0 to 1
     * @return scalar change per unit of normalized parameter
     */
    public double derivative(double t) {
        double a = coeffs.get(0, 0);
        double b = coeffs.get(0, 1);
        double c = coeffs.get(0, 2);
        return 3 * a * Math.pow(t,2) + 2 * b * t + c;
    }
    /** Returns this curve's polynomial coefficients.
     * @return 1x4 matrix containing [a, b, c, d] for {@code f(t) = a*t^3 + b*t^2 + c*t + d} */
    public SimpleMatrix getCoeffs(){
        return coeffs;
    }
    /**
     * Computes uniform Catmull-Rom coefficients [a, b, c, d] for a scalar dimension.
     * @param P0 value before the segment start, in the caller's units
     * @param P1 value at the segment start
     * @param P2 value at the segment end
     * @param P3 value after the segment end
     * @return 1x4 matrix containing coefficients for a cubic over normalized parameter [0, 1]
     */
    private SimpleMatrix catmullRomCoeffs(double P0, double P1, double P2, double P3) {
        double d = P1;
        double c = 0.5 * (P2 - P0);
        double b = 0.5 * (2*P0 - 5*P1 + 4*P2 - P3);
        double a = 0.5 * (-P0 + 3*P1 - 3*P2 + P3);
        return new SimpleMatrix(new double[][]{{a, b, c, d}});
    }
}
