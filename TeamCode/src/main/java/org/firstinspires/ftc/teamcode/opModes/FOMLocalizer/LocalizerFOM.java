package org.firstinspires.ftc.teamcode.opModes.FOMLocalizer;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

public class localizerFOM implements Localizer {
    @Override
    public Pose getPose() {
        return null;
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {

    }

    @Override
    public void setPose(Pose setPose) {

    }

    @Override
    public void update() {

    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    @Override
    public double getIMUHeading() {
        return 0;
    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
