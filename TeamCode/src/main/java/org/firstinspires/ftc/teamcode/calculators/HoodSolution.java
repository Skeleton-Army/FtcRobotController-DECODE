package org.firstinspires.ftc.teamcode.calculators;

public class HoodSolution {
    private double hoodAngle;
    private boolean isValid;

    public HoodSolution(double hoodAngle, boolean isValid) {
        this.hoodAngle = hoodAngle;
        this.isValid = isValid;
    }
    public double getHoodAngle() {
        return hoodAngle;
    }

    public boolean isValid() {
        return isValid;
    }
}
