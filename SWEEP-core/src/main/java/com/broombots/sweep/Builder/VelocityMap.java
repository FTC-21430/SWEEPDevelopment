package com.broombots.sweep.Builder;

import com.broombots.sweep.Classes.Coordinate;

import java.util.ArrayList;
import java.util.LinkedList;

public class VelocityMap {
    private LinkedList<MovementPoint> rawVelocityProfile = new LinkedList<>();
    private LinkedList<Double> time = new LinkedList<>();
    private double initialKey, samplesRate;
    private MovementPoint startPoint, endPoint;

    private ArrayList<MovementPoint> normalizedVelocityProfile = new ArrayList<>();
    private ArrayList<Double> normalizedDistanceProfile = new ArrayList<>();
    public VelocityMap(double initialKey, double sampleRate, MovementPoint startPoint, MovementPoint endPoint){
        this.initialKey = initialKey;
        this.samplesRate = sampleRate;
        this.startPoint = startPoint;
        this.endPoint = endPoint;
    }
    public static MovementMap generateMovementFromVelocityMaps(VelocityMap[] velocityMaps, double sampleRate){
        if (velocityMaps == null || velocityMaps.length == 0) throw new IllegalArgumentException("Cannot generate movement from no velocity maps");
        if (sampleRate <= 0.0) throw new IllegalArgumentException("sampleRate must be positive");
        MovementMap combined = new MovementMap(sampleRate);
        double maxDistance = 0;
        for (VelocityMap map : velocityMaps){
            if (map == null) throw new IllegalArgumentException("Velocity map array contains a null entry");
            map.normalizeVelocityProfile();
            maxDistance = Math.max(maxDistance, map.getMaxDistance());
        }

        // TODO: this picks, independently at each distance sample, whichever envelope (start-anchored,
        // the two curvature-anchored passes, end-anchored) currently has the lowest velocity magnitude,
        // and takes that envelope's ENTIRE MovementPoint (position + velocity + acceleration) wholesale.
        // Each envelope is internally kinematically consistent on its own, but nothing guarantees adjacent
        // *combined* samples came from the same envelope -- if the winning envelope switches between d and
        // d+sampleRate, the resulting position/velocity/acceleration can have a seam (small discontinuity)
        // right where it matters most: entering/exiting curvature-limited zones. Revisit once the ideal-map
        // (calculateIdealMovementMap) and comeToStop boundary handling are both correct -- may shrink or
        // disappear once the envelopes agree more closely; if not, consider re-deriving acceleration from
        // the combined position/velocity samples after this min-selection instead of carrying it over raw.
        for (double d = 0.0; d <= maxDistance + 1e-9; d += sampleRate){
            MovementPoint smallestPoint = velocityMaps[0].getPointAtDistance(d);
            for (int j = 1; j < velocityMaps.length; j++){
                MovementPoint point = velocityMaps[j].getPointAtDistance(d);
                smallestPoint = smallestPoint.getVelocityMagnitude() > point.getVelocityMagnitude()?  point : smallestPoint;
            }
            combined.addMovementPoint(smallestPoint);
        }

        return combined;
    }
    void normalizeVelocityProfile() {
        if (samplesRate <= 0.0) throw new IllegalStateException("samplesRate must be positive");
        // TODO: startPoint/endPoint are spliced onto every envelope's raw profile unconditionally, but the
        // forward pass (up to endingDistance) and backward pass (down to startingDistance) inside
        // MotionProfileProcessor.compileVelocityProfile already make each envelope span the FULL segment
        // range on its own. Splicing the same global start/end point onto all 4 envelopes regardless of
        // anchor "point" may duplicate or conflict with the naturally-simulated boundary sample already
        // present in rawVelocityProfile -- double check this doesn't introduce an extra/inconsistent sample
        // at either end, especially once comeToStop's zero-velocity endPoint (see compileVelocityProfile
        // TODO) is threaded through correctly.
        rawVelocityProfile.addFirst(startPoint);
        rawVelocityProfile.add(endPoint);
        MovementPoint[] raw = rawVelocityProfile.toArray(new MovementPoint[0]);
        if (raw.length == 0) return;
        if (raw.length == 1) {
            normalizedVelocityProfile.clear();
            normalizedDistanceProfile.clear();
            normalizedVelocityProfile.add(raw[0]);
            normalizedDistanceProfile.add(initialKey);
            return;
        }

        // Build cumulative arc-length list, anchored at initialKey
        ArrayList<Double> cumDist = new ArrayList<>();
        cumDist.add(initialKey);
        for (int i = 1; i < raw.length; i++) {
            cumDist.add(cumDist.get(i - 1) +
                Coordinate.getDistanceBetweenCoordinates(
                    raw[i - 1].getPosition(),
                    raw[i].getPosition()
                ));
        }

        double totalDistance = cumDist.get(raw.length - 1);
        normalizedVelocityProfile.clear();
        normalizedDistanceProfile.clear();

        for (double target = initialKey; target <= totalDistance + 1e-9; target += samplesRate) {
            // Binary search: find largest index whose cumDist <= target
            int lo = 0;
            int hi = raw.length - 1;
            while (lo < hi - 1) {
                int mid = (lo + hi) / 2;
                if (cumDist.get(mid) <= target) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }

            double range = cumDist.get(hi) - cumDist.get(lo);
            double t = (range > 1e-9) ? (target - cumDist.get(lo)) / range : 0.0;
            MovementPoint interpolated = MovementMap.lerpMovementPoint(raw[lo], raw[hi], t);

            normalizedVelocityProfile.add(interpolated);
            normalizedDistanceProfile.add(target);
        }
    }
    public void addForwardPoint(MovementPoint point){
        rawVelocityProfile.add(point);
    }

    /**
     * Returns the interpolated MovementPoint at the given distance along the
     * normalized velocity profile. Binary search locates the bracket in
     * {@code normalizedDistanceProfile}, then lerps between the two surrounding points.
     *
     * @param distance the arc-length distance to query, in the same units as initialKey
     * @return the interpolated MovementPoint at that distance
     */
    public MovementPoint getPointAtDistance(double distance) {
        if (normalizedVelocityProfile.isEmpty()) throw new RuntimeException("Normalized velocity profile is empty; call normalizeVelocityProfile() first");

        // Clamp to profile bounds
        if (distance <= normalizedDistanceProfile.get(0)) return normalizedVelocityProfile.get(0);
        int lastIdx = normalizedDistanceProfile.size() - 1;
        if (distance >= normalizedDistanceProfile.get(lastIdx)) return normalizedVelocityProfile.get(lastIdx);

        // Binary search: find largest index whose distance <= target
        int lo = 0;
        int hi = lastIdx;
        while (lo < hi - 1) {
            int mid = (lo + hi) / 2;
            if (normalizedDistanceProfile.get(mid) <= distance) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        double range = normalizedDistanceProfile.get(hi) - normalizedDistanceProfile.get(lo);
        double t = (range > 1e-9) ? (distance - normalizedDistanceProfile.get(lo)) / range : 0.0;
        return MovementMap.lerpMovementPoint(normalizedVelocityProfile.get(lo), normalizedVelocityProfile.get(hi), t);
    }
    public void addBackPassPoint(MovementPoint point){
        rawVelocityProfile.addFirst(point);
    }
    public double getMaxDistance(){
        if (normalizedDistanceProfile.isEmpty()) throw new RuntimeException("Normalized velocity profile not calculated");
        return normalizedDistanceProfile.get(normalizedDistanceProfile.size()-1);
    }
}
