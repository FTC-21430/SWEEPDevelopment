package com.broombots.sweep.Classes;

/**
 * Tracks elapsed time while using SWEEP. Uses System.nanoTime() so it works on every Android API level.
 */
public class SWEEPTimer {
    /** Time of the last reset, in nanoseconds. */
    private long startNanos;

    public SWEEPTimer(){
        reset();
    }

    /**
     * @return seconds since the last reset, including the fractional part
     */
    public double getSeconds(){
        return (System.nanoTime() - startNanos) / 1e9;
    }

    /**
     * @return milliseconds since the last reset, including the fractional part
     */
    public double getMilliSeconds(){
        return (System.nanoTime() - startNanos) / 1e6;
    }

    public void reset(){
        startNanos = System.nanoTime();
    }
}
