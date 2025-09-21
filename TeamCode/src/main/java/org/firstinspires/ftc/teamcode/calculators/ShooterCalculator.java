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

    //These are placeholders for the real values.
    //TODO: Find the real values
    private final double shooterMinVelocity = -1;
    private final double shooterMaxVelocity = -1;
    private final double minDistance = 0.7;
    private final double maxDistance = 3.5;

    private double shooterVelocity(double distance) {
        return distance * (shooterMaxVelocity - shooterMinVelocity) / (maxDistance - minDistance);
    }

    public ShooterCalculator() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        this.model = mapper.readValue(new File(coefficientsFilePath), Model.class);
    }

    /**
     *
     * @param metersPerSecond Starting velocity in meters/second
     * @return Motor RPM
     */
    public double convertVelocityToMotorRPM(double metersPerSecond) {
        //TODO: Find convertion with regression (probably linear but could be parabolic)
        return -1;
    }

    private double approximateStaticVerticalAngle(double distanceFromGoal) {
        double result = 0.0;
        for (int exponent = 0; exponent < model.powers.length; exponent++) {
            result += model.coefficients[exponent] * Math.pow(distanceFromGoal, exponent);
        }
        return result;
    }

    /**
     *
     * @param robotPose Robot position (pedro field coordinates)
     * @param goalPose Goal position (pedro field coordinates)
     * @param velocity Robot velocity vector in meters per second
     * @return ShootingSolution at field coordinates (e.g. horizontal angle at relative to the field and not the robot)
     */
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector3D velocity) {
        double distanceFromGoal = robotPose.distanceFrom(goalPose);
        double staticVerticalAngle = approximateStaticVerticalAngle(distanceFromGoal);
        double horizontalAngle = Math.atan2(goalPose.getX() - robotPose.getX(), goalPose.getY() - robotPose.getY());
        Vector3D targetVelocityBase = new Vector3D(staticVerticalAngle, horizontalAngle);
        Vector3D targetVelocity = new Vector3D(shooterVelocity(distanceFromGoal), targetVelocityBase);
        Vector3D v0 = targetVelocity.add(velocity.negate());

        return new ShootingSolution(v0.getDelta(), v0.getAlpha(), v0.getNorm()); //might be mistake from wrong placement of alpha and delta
    }
}
