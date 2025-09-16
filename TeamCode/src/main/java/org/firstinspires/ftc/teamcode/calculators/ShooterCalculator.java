package org.firstinspires.ftc.teamcode.calculators;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.geometry.Pose;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import java.io.File;
import java.io.IOException;

public class ShooterCalculator implements IShooterCalculator {
    static class Model {
        public double[] coefficients;
        public double[] powers;
    }

    private final Model model;
    private static final String coefficientsFilePath = "Coefficients.json";
    private final ObjectMapper mapper = new ObjectMapper();

    private final double shooterVelocity = 2;

    public ShooterCalculator() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        this.model = mapper.readValue(new File(coefficientsFilePath), Model.class);
    }

    private double approximateStaticVerticalAngle(double distanceFromGoal) {
        double result = 0.0;
        for (int exponent = 0; exponent < model.powers.length; exponent++) {
            result += model.coefficients[exponent] * Math.pow(distanceFromGoal, exponent);
        }
        return result;
    }

    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector3D velocity) {
        double distanceFromGoal = robotPose.distanceFrom(goalPose);
        double staticVerticalAngle = approximateStaticVerticalAngle(distanceFromGoal);
        double horizontalAngle = Math.atan2(goalPose.getX() - robotPose.getX(), goalPose.getY() - robotPose.getY());
        Vector3D targetVelocityBase = new Vector3D(staticVerticalAngle, horizontalAngle);
        Vector3D targetVelocity = new Vector3D(shooterVelocity, targetVelocityBase);
        Vector3D v0 = targetVelocity.add(velocity.negate());

        return new ShootingSolution(v0.getDelta(), v0.getAlpha(), v0.getNorm()); //might be mistake from wrong placement of alpha and delta
    }
}
