package com.example.plot;

import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Builder.SequenceBuilder;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Waypoint;
import com.broombots.sweep.Defaults.DefaultRobotMovementParameters;
import com.github.sh0nk.matplotlib4j.NumpyUtils;
import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonConfig;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;
import com.broombots.sweep.Splines.Segments.FollowSplineSegment;
import com.broombots.sweep.Splines.SplineWaypoint;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import java.util.stream.Collectors;

public class Plotting {
    private static final String LOCAL_VENV_PYTHON = "/home/tobin/SWEEPDevelopment/.venv/bin/python3";

    public static void main(String[] args) throws PythonExecutionException, IOException {

        PathPoint start = new PathPoint();
        start.position = new Pos2D(-20,10,0);
        start.velocity = new Pos2D(0,0,0);
        start.time = 0;

        Sequence sequence = new SequenceBuilder(new DefaultRobotMovementParameters())
                .start(start)
                .splineToAngle(0,20,0, 1)
                .splineToAngle(20,20,0, 1)
//                .splineToAngle(20,60,0, 1)
                .splineToAngle(-20,20,0, 1)
//                .splineToAngle(0,0,0, 1)
                .end(40, 0, 0)
                .build();
        SWEEPFullPlotFullRender.PlotRender(sequence, 0.01, LOCAL_VENV_PYTHON);
//        List<Coordinate> waypoints = Arrays.asList(
//                new Coordinate(0, 0),
//                new Coordinate(0, 40)
//        );
//        SweepCatmullRomPathPlotter.plotPath(waypoints, 40, LOCAL_VENV_PYTHON);
    }

    public static void makePlot() throws PythonExecutionException, IOException {
        List<Double> x = NumpyUtils.linspace(-Math.PI, Math.PI, 256);
        List<Double> c = x.stream().map(Math::cos).collect(Collectors.toList());
        List<Double> s = x.stream().map(Math::sin).collect(Collectors.toList());

        Plot plt = Plot.create(PythonConfig.pythonBinPathConfig(resolvePythonBinPath()));
        plt.plot().add(x,c);
        plt.plot().add(x,s);
        plt.show();
    }


    private static String resolvePythonBinPath() {
        String virtualEnv = System.getenv("VIRTUAL_ENV");
        if (virtualEnv != null && !virtualEnv.isEmpty()) {
            File venvPython = new File(virtualEnv, "bin/python3");
            if (venvPython.exists()) return venvPython.getAbsolutePath();
        }

        File localPython = new File(LOCAL_VENV_PYTHON);
        return localPython.exists() ? localPython.getAbsolutePath() : "python3";
    }
}

class SweepCatmullRomPathPlotter {

    /**
     * Plots a continuous SWEEP Catmull-Rom XY path built from waypoint anchors.
     *
     * @param waypoints anchor positions in field units (inches)
     * @param samplesPerSegment number of interpolation samples per segment
     * @param pythonBinPath python interpreter path for matplotlib4j
     */
    public static void plotPath(List<Pos2D> waypoints, int samplesPerSegment, String pythonBinPath)
            throws PythonExecutionException, IOException {
        if (waypoints == null || waypoints.size() < 2) return;

        int safeSamplesPerSegment = samplesPerSegment > 1 ? samplesPerSegment : 40;
        List<SplineWaypoint> splineWaypoints = waypoints.stream()
                .map(point -> new SplineWaypoint(point, 1.0))
                .collect(Collectors.toList());

        List<Double> xPath = new ArrayList<>();
        List<Double> yPath = new ArrayList<>();

        for (int index = 0; index < splineWaypoints.size() - 1; index++) {
            Waypoint p1 = splineWaypoints.get(Math.max(0, index - 1));
            Waypoint p2 = splineWaypoints.get(index);
            Waypoint p3 = splineWaypoints.get(index + 1);
            Waypoint p4 = splineWaypoints.get(Math.min(splineWaypoints.size() - 1, index + 2));

            FollowSplineSegment segment = new FollowSplineSegment(p1, p2, p3, p4);
            for (int sample = 0; sample < safeSamplesPerSegment; sample++) {
                double t = (double) sample / safeSamplesPerSegment;
                Pos2D point = segment.getPosition(t);
                xPath.add(point.x);
                yPath.add(point.y);
            }
        }

        Pos2D end = waypoints.get(waypoints.size() - 1);
        xPath.add(end.x);
        yPath.add(end.y);

        Plot plt = Plot.create(PythonConfig.pythonBinPathConfig(pythonBinPath));
        plt.plot().add(xPath, yPath).label("SWEEP Catmull-Rom Path");
        plt.legend();
        plt.show();
    }
}
class SWEEPFullPlotFullRender{
    private static final String[] CORRESPONDENCE_COLORS = new String[]{
            "#e6194b", "#3cb44b", "#ffe119", "#4363d8", "#f58231",
            "#911eb4", "#46f0f0", "#f032e6"
    };

    public static void PlotRender(Sequence compiledSequence, double timeSampleRate, String pythonDirectory)
            throws PythonExecutionException, IOException {
        ArrayList<Double> timeValues = new ArrayList<>();
        ArrayList<Double> xValues = new ArrayList<>();
        ArrayList<Double> yValues = new ArrayList<>();
        ArrayList<Double> angleValues = new ArrayList<>();
        ArrayList<Double> xVelValues = new ArrayList<>();
        ArrayList<Double> yVelValues = new ArrayList<>();
        ArrayList<Double> angleVelValues = new ArrayList<>();
        ArrayList<Double> velMagnitude = new ArrayList<>();
        System.out.println("CompiledSequence last time is: " + compiledSequence.getLastTime());
        for (double i = 0; i < compiledSequence.getLastTime(); i += timeSampleRate){
            double velX = compiledSequence.getMovement(i).velocity.x;
            double velY = compiledSequence.getMovement(i).velocity.y;
            xValues.add(compiledSequence.getMovement(i).position.x);
            yValues.add(compiledSequence.getMovement(i).position.y);
            angleValues.add(compiledSequence.getMovement(i).position.angle);
            xVelValues.add(velX);
            yVelValues.add(velY);
            angleVelValues.add(compiledSequence.getMovement(i).velocity.angle);
            velMagnitude.add(compiledSequence.getMovement(i).velocity.getMagnitude());
            timeValues.add(i);
        }

        // All plots in one figure so all windows open at once
        Plot plt = Plot.create(PythonConfig.pythonBinPathConfig(pythonDirectory));

        plt.subplot(3, 3, 1);
        plt.plot().add(xValues, yValues).label("XY Path");
        addCorrespondenceMarkers(plt, xValues, yValues);
        plt.title("XY Path");
        plt.xlabel("X (in)");
        plt.ylabel("Y (in)");
        plt.legend();

        plt.subplot(3, 3, 2);
        plt.plot().add(timeValues, angleValues).label("Angle");
        plt.title("Angle vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("Angle (deg)");
        plt.legend();

        plt.subplot(3, 3, 3);
        plt.plot().add(timeValues, velMagnitude).label("Speed");
        addCorrespondenceMarkers(plt, timeValues, velMagnitude);
        plt.title("Speed vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("Speed (in/s)");
        plt.legend();

        plt.subplot(3, 3, 4);
        plt.plot().add(timeValues, xValues).label("X Position");
        plt.title("X Position vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("X (in)");
        plt.legend();

        plt.subplot(3, 3, 5);
        plt.plot().add(timeValues, yValues).label("Y Position");
        plt.title("Y Position vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("Y (in)");
        plt.legend();

        plt.subplot(3, 3, 7);
        plt.plot().add(timeValues, xVelValues).label("X Velocity");
        addCorrespondenceMarkers(plt, timeValues, xVelValues);
        plt.title("Robot Relative X Velocity vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("Vel X (in/s)");
        plt.legend();

        plt.subplot(3, 3, 8);
        plt.plot().add(timeValues, yVelValues).label("Y Velocity");
        addCorrespondenceMarkers(plt, timeValues, yVelValues);
        plt.title("Robot Relative Y Velocity vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("Vel Y (in/s)");
        plt.legend();

        plt.subplot(3, 3, 9);
        plt.plot().add(timeValues, angleVelValues).label("Angular Velocity");
        plt.title("Angular Velocity vs Time");
        plt.xlabel("Time (s)");
        plt.ylabel("Vel Angle (deg/s)");
        plt.legend();

        plt.show();
    }

    private static void addCorrespondenceMarkers(Plot plt, List<Double> xValues, List<Double> yValues) {
        if (xValues == null || yValues == null) return;
        if (xValues.size() != yValues.size() || xValues.isEmpty()) return;

        int markerCount = Math.min(CORRESPONDENCE_COLORS.length, xValues.size());
        if (markerCount == 0) return;

        int lastIndex = xValues.size() - 1;
        for (int i = 0; i < markerCount; i++) {
            int index = markerCount == 1
                    ? 0
                    : (int) Math.round((double) i * lastIndex / (markerCount - 1));
            plt.plot()
                    .add(Collections.singletonList(xValues.get(index)), Collections.singletonList(yValues.get(index)), "o")
                    .color(CORRESPONDENCE_COLORS[i]);
        }
    }
}
