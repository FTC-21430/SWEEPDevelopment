package com.broombots.sweep.Classes;

/** Measures elapsed wall-clock time using the monotonic {@link System#nanoTime()} source. */
public class SWEEPTimer {
    /** Time of the last reset, in nanoseconds. */
    private long startNanos;

    /** Creates a timer and starts measuring immediately. */
    public SWEEPTimer(){
        reset();
    }

    /** Returns the elapsed time since construction or the last reset.
     * @return elapsed time in seconds, including fractional seconds
     */
    public double getSeconds(){
        return (System.nanoTime() - startNanos) / 1e9;
    }

    /** Returns the elapsed time since construction or the last reset.
     * @return elapsed time in milliseconds, including fractional milliseconds
     */
    public double getMilliSeconds(){
        return (System.nanoTime() - startNanos) / 1e6;
    }

    /** Restarts elapsed-time measurement from the current instant. */
    public void reset(){
        startNanos = System.nanoTime();
    }
}
