package com.broombots.sweep.Classes;

import org.ejml.simple.SimpleMatrix;

public class CatmullRomCubic {
    private final SimpleMatrix coeffs;
    public CatmullRomCubic(double point1, double point2, double point3, double point4) {
        coeffs = new SimpleMatrix(catmullRomCoeffs(point1, point2, point3, point4));
    }
    public double evaluate(double t) {
        double a = coeffs.get(0, 0);
        double b = coeffs.get(0, 1);
        double c = coeffs.get(0, 2);
        double d = coeffs.get(0, 3);
        return a * Math.pow(t,3) + b * Math.pow(t,2) + c * t + d;
    }
    /**
     * Compute the derivative of the polynomial with respect to absolute time at the given
     * relative time. Uses the chain rule: d/dt = (d/dτ) * (1/timeScalar).
     */
    public double derivative(double t) {
        double a = coeffs.get(0, 0);
        double b = coeffs.get(0, 1);
        double c = coeffs.get(0, 2);
        return 3 * a * Math.pow(t,2) + 2 * b * t + c;
    }
    public SimpleMatrix getCoeffs(){
        return coeffs;
    }
    /**
     * Compute uniform Catmull-Rom coefficients [a, b, c, d] for a single scalar dimension,
     * returned as a 1x4 SimpleMatrix so CubicPolynomial can unpack them directly.
     * Uses the standard alpha=0 (uniform) formula operating on scalar values independently —
     * no cross-axis distances involved, which keeps each dimension clean.
     *
     * @param P0 value at the point before the segment start (for tangent)
     * @param P1 value at the segment start
     * @param P2 value at the segment end
     * @param P3 value at the point after the segment end (for tangent)
     * @return 1x4 SimpleMatrix {a, b, c, d} such that f(t) = a*t^3 + b*t^2 + c*t + d for t in [0,1]
     */
    private SimpleMatrix catmullRomCoeffs(double P0, double P1, double P2, double P3) {
        double d = P1;
        double c = 0.5 * (P2 - P0);
        double b = 0.5 * (2*P0 - 5*P1 + 4*P2 - P3);
        double a = 0.5 * (-P0 + 3*P1 - 3*P2 + P3);
        return new SimpleMatrix(new double[][]{{a, b, c, d}});
    }
}
