package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;

public class Kinematics {
    private double lastAngularVel = 0;
    private Vector lastLinearVel = new Vector();
    private long lastUpdateTime = 0;

    private double filteredAngularAccel = 0;
    private Vector filteredLinearAccel = new Vector();

    public Kinematics() {
        reset();
    }

    public void reset() {
        lastUpdateTime = 0;
        lastAngularVel = 0;
        lastLinearVel = new Vector();
        filteredAngularAccel = 0;
        filteredLinearAccel = new Vector();
    }

    public void update(PoseTracker tracker, double gain) {
        long currentTime = System.nanoTime();

        if (lastUpdateTime == 0) {
            lastUpdateTime = currentTime;
            lastAngularVel = tracker.getAngularVelocity();
            lastLinearVel = tracker.getVelocity();
            return;
        }

        double dt = (currentTime - lastUpdateTime) / 1e9;
        if (dt <= 1e-6) return;

        // 1. Angular Acceleration
        double currentAngularVel = tracker.getAngularVelocity();
        double rawAngularAccel = (currentAngularVel - lastAngularVel) / dt;
        filteredAngularAccel = lowPassFilter(rawAngularAccel, filteredAngularAccel, gain);

        // 2. Linear Acceleration
        Vector currentLinearVel = tracker.getVelocity();
        Vector rawLinearAccel = currentLinearVel.minus(lastLinearVel);
        rawLinearAccel.times(1.0 / dt);
        filteredLinearAccel = lowPassFilter(rawLinearAccel, filteredLinearAccel, gain);

        lastAngularVel = currentAngularVel;
        lastLinearVel = currentLinearVel;
        lastUpdateTime = currentTime;
    }

    public static double lowPassFilter(double newVal, double oldVal, double gain) {
        return (gain * newVal) + ((1.0 - gain) * oldVal);
    }

    public static Vector lowPassFilter(Vector newVal, Vector oldVal, double gain) {
        Vector newComp = newVal.copy();
        newComp.times(gain);
        Vector oldComp = oldVal.copy();
        oldComp.times(1.0 - gain);
        return newComp.plus(oldComp);
    }

    public double getAngularAcceleration() { return filteredAngularAccel; }
    public Vector getLinearAcceleration() { return filteredLinearAccel; }
}
