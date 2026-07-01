package org.firstinspires.ftc.teamcode.calculators;

public class ShootingSolution {
    private final double horizontalAngle;
    private final double verticalAngle;
    private final double flywheelRPM;
    private final boolean canShoot;
    private final double exitVel;
    private final double angularVelocity;
    private final double angularAcceleration;

    public ShootingSolution(double horizontalAngle, double verticalAngle, double flywheelRPM, boolean canShoot, double exitVel, double angularVelocity, double angularAcceleration) {
        this.horizontalAngle = horizontalAngle;
        this.verticalAngle = verticalAngle;
        this.flywheelRPM = flywheelRPM;
        this.canShoot = canShoot;
        this.exitVel = exitVel;
        this.angularVelocity = angularVelocity;
        this.angularAcceleration = angularAcceleration;
    }

    public ShootingSolution(double horizontalAngle, double verticalAngle, double flywheelRPM, double exitVel, double angularVelocity, double angularAcceleration) {
        this(horizontalAngle, verticalAngle, flywheelRPM, true, exitVel, angularVelocity, angularAcceleration);
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
    public double getExitVel() {
        return exitVel;
    }
    public boolean getCanShoot() {
        return canShoot;
    }
    public double getAngularVelocity() {
        return angularVelocity;
    }
    public double getAngularAcceleration() {
        return angularAcceleration;
    }
}
