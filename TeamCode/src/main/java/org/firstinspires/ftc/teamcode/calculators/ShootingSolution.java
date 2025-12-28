package org.firstinspires.ftc.teamcode.calculators;

public class ShootingSolution {
    private final double horizontalAngle;
    private final double verticalAngle;
    private final double flywheelRPM;

    public ShootingSolution(double horizontalAngle, double verticalAngle, double flywheelRPM) {
        this.horizontalAngle = horizontalAngle;
        this.verticalAngle = verticalAngle;
        this.flywheelRPM = flywheelRPM;
    }

    public double getHorizontalAngle() {
        return horizontalAngle;
    }
    public double getVerticalAngle() {
        return verticalAngle;
    }
    public double getRPM() {
        return flywheelRPM;
    }
}
