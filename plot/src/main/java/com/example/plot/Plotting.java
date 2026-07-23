package com.example.plot;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.Waypoint;
import com.github.sh0nk.matplotlib4j.NumpyUtils;
import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonConfig;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;
import com.broombots.sweep.Splines.Segments.FollowSplineSegment;
import com.broombots.sweep.Splines.SplineWaypoint;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class Plotting {
    private static final String LOCAL_VENV_PYTHON = "/home/tobin/SWEEPDevelopment/.venv/bin/python3";

    public static void main(String[] args) throws PythonExecutionException, IOException {
//        makePlot();

        List<Coordinate> sweepWaypoints = Arrays.asList(
                new Coordinate(0.0, 0.0),
                new Coordinate(8.0, 20.0),
                new Coordinate(24.0, 12.0),
                new Coordinate(40.0, 10.0),
                new Coordinate(0.0, 22.0)
        );
        SweepCatmullRomPathPlotter.plotPath(sweepWaypoints, 60, resolvePythonBinPath());
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
    public static void plotPath(List<Coordinate> waypoints, int samplesPerSegment, String pythonBinPath)
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
                Coordinate point = segment.getPosition(t);
                xPath.add(point.getX());
                yPath.add(point.getY());
            }
        }

        Coordinate end = waypoints.get(waypoints.size() - 1);
        xPath.add(end.getX());
        yPath.add(end.getY());

        Plot plt = Plot.create(PythonConfig.pythonBinPathConfig(pythonBinPath));
        plt.plot().add(xPath, yPath).label("SWEEP Catmull-Rom Path");
        plt.legend();
        plt.show();
    }
}
