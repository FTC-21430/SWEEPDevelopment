package com.broombots.sweep.Classes;

import android.os.Build;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;

// Track the passage of time while using SWEEP.
public class Timer {
    private Instant startTime;
    public Timer(){
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) throw new RuntimeException("API Version is too low to use SWEEP Lib");
        reset();
    }
    public double getSeconds(){
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) throw new RuntimeException("API Version is too low to use SWEEP Lib");
        return getDuration().getSeconds();
    }
    public double getMilliSeconds(){
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) throw new RuntimeException("API Version is too low to use SWEEP Lib");
        return getDuration().get(ChronoUnit.MILLIS);
    }
    private Duration getDuration(){
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) throw new RuntimeException("API Version is too low to use SWEEP Lib");
        return Duration.between(startTime, Instant.now());
    }
    public void reset(){
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) throw new RuntimeException("API Version is too low to use SWEEP Lib");
        startTime = Instant.now();
    }
}

