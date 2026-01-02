package org.firstinspires.ftc.teamcode.calculators;

public class ShootingSolution {
    private final double horizontalAngle;
    private final double verticalAngle;
    private final double flywheelRPM;
    private final boolean canShoot;

    public ShootingSolution(double horizontalAngle, double verticalAngle, double flywheelRPM, boolean canShoot) {
        this.horizontalAngle = horizontalAngle;
        this.verticalAngle = verticalAngle;
        this.flywheelRPM = flywheelRPM;
        this.canShoot = canShoot;
    }

    public ShootingSolution(double horizontalAngle, double verticalAngle, double flywheelRPM) {
        this(horizontalAngle, verticalAngle, flywheelRPM, true);
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
    public boolean getCanShoot() {
        return canShoot;
    }
}
