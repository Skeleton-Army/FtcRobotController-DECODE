package org.firstinspires.ftc.teamcode.calculators;

public class ShootingSolution {
    private final double horizontalAngle;
    private final double verticalAngle;
    private final double flywheelRPM;
    private final double exitVelocity;

    public ShootingSolution(double horizontalAngle, double verticalAngle, double flywheelRPM, double exitVelocity) {
        this.horizontalAngle = horizontalAngle;
        this.verticalAngle = verticalAngle;
        this.flywheelRPM = flywheelRPM;
        this.exitVelocity = exitVelocity;
    }

    public double getHorizontalAngle() {
        return horizontalAngle;
    }
    public double getVerticalAngle() {
        return verticalAngle;
    }
    public double getVelocity() {
        return flywheelRPM;
    }
    public double getVelocityMetersPerSec() {
        return exitVelocity;
    }
}
