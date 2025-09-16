package org.firstinspires.ftc.teamcode.calculators;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.io.File;
import java.io.IOException;

public class ShooterCalculator implements IShooterCalculator {
    static class Model {
        public double[] coefficients;
        public double[] powers;
    }
    private final Model model;
    private final String coefficientsFilePath = "Coefficients.json";
    private final ObjectMapper mapper = new ObjectMapper();
    public ShooterCalculator() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        this.model = mapper.readValue(new File(coefficientsFilePath), Model.class);
    }
    private double approximateStaticVerticalAngle(double distanceFromGoal)
    {
        double result = 0.0;
        for (int exponent = 0; exponent < model.powers.length; exponent++) {
            result += model.coefficients[exponent] * Math.pow(distanceFromGoal, exponent);
        }
        return result;
    }
    public ShootingSolution getShootingSolution(Pose robotPose, Vector goalVelocity) {
        // THIS IS A PLACEHOLDER FOR THE ACTUAL SHOOTER CALCULATOR
        //TODO: Implement the actual shooter calculator
        return new ShootingSolution(Math.PI / 2, Math.PI / 2, 1000);
    }
}
