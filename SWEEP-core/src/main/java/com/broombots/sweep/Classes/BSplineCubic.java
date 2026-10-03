package com.broombots.sweep.Classes;

import org.ejml.simple.SimpleMatrix;

public class BSplineCubic {
    private final SimpleMatrix controlPoints;
    public BSplineCubic(double point1, double point2, double point3, double point4){
        controlPoints = new SimpleMatrix(new double[][]{
            {point1, point2, point3, point4}
        });
    }
    private double basis(int i, double t) {
        switch (i) {
            case 0:
                return ((-t * t * t + 3 * t * t - 3 * t + 1) / 6.0);
            case 1:
                return ((3 * t * t * t - 6 * t * t + 4) / 6.0);       // B_1(t) = (3t^3 - 6t^2 + 4) / 6
            case 2:
                return ((-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0); // B_2(t) = (-3t^3 + 3t^2 + 3t + 1) / 6
            case 3:
                return ((t * t * t) / 6.0);                           // B_3(t) = t^3 / 6
            default:
                return 0.0;
        }
    }
    public double evaluate(double t){
        return basis(0, t) * controlPoints.get(0) +
                basis(1,t) * controlPoints.get(1) +
                basis(2,t) * controlPoints.get(2) +
                basis(3,t) * controlPoints.get(3);
    }
}
