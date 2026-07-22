package com.example.plot;

import com.github.sh0nk.matplotlib4j.NumpyUtils;
import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonConfig;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

public class Plotting {
    private static final String LOCAL_VENV_PYTHON = "/home/tobin/SWEEPDevelopment/.venv/bin/python3";

    public static void main(String[] args) throws PythonExecutionException, IOException {
        makePlot();
    }

    public static void makePlot() throws PythonExecutionException, IOException {
        List<Double> x = NumpyUtils.linspace(-Math.PI, Math.PI, 256);
        List<Double> c = x.stream().map(xi -> Math.cos(xi)).collect(Collectors.toList());
        List<Double> s = x.stream().map(xi -> Math.sin(xi)).collect(Collectors.toList());

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