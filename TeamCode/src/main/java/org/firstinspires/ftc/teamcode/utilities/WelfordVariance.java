package org.firstinspires.ftc.teamcode.utilities;

public final class WelfordVariance {
    private int n = 0;
    private double mean = 0;
    private double m2 = 0;

    public void update(double x) {
        n++;
        double delta = x - mean;
        mean += delta / n;
        double delta2 = x - mean;
        m2 += delta * delta2;
    }

    public double variance() {
        return m2 / (n - 1);
    }

    public double mean() {
        return mean;
    }

    public int n() {
        return n;
    }

    public double stdDev() {
        return Math.sqrt(variance());
    }
}