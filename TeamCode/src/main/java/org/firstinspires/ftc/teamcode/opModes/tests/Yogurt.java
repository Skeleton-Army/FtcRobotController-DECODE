package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.geometry.Pose;

public class Yogurt {
    private final double sampleTime;
    private final Pose pose;
    public Yogurt(double runtime, Pose botPose) {
        this.sampleTime = runtime;
        this.pose = botPose;
    }

    public Pose getPose() {
        return pose;
    }

    public double getSampleTime() {
        return sampleTime;
    }

    public double getX() {
        return getPose().getX();
    }

    public double getY() {
        return getPose().getY();
    }

    public double getHead() {
        return getPose().getHeading();
    }
}
