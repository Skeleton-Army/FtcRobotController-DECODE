package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.junit.jupiter.api.Test;

public class ShooterTurretAngleTest {

    private static final double EPS = 1e-3;

    private void setGoal(double x, double y) {
        ShooterConfig.GOAL_X = x;
        ShooterConfig.GOAL_Y = y;
        ShooterConfig.DISTANCE_TO_BOT_CENTER = 0;
    }

    // -------------------
    // GOAL at (0, 144)
    // -------------------

    @Test
    void testFacingStraightAtGoal_originGoal() {
        setGoal(0, 144);

        double angle = Shooter.getTurretAngle(0, 0, Math.PI / 2);
        assertEquals(0.0, angle, EPS);
    }

    @Test
    void testRobotToLeftOfGoal_originGoal() {
        setGoal(0, 144);

        double angle = Shooter.getTurretAngle(-24, 0, Math.PI / 2);
        assertTrue(angle > 0);
    }

    @Test
    void testRobotBehindGoal_originGoal() {
        setGoal(0, 144);

        double angle = Shooter.getTurretAngle(0, 180, -Math.PI / 2);
        assertEquals(Math.PI, Math.abs(angle), 1e-1);
    }

    // -------------------
    // GOAL at (144, 144)
    // -------------------

    @Test
    void testFacingGoal_diagonalGoal() {
        setGoal(144, 144);

        double angle = Shooter.getTurretAngle(144, 0, Math.PI / 2);
        assertEquals(0.0, angle, EPS);
    }

    @Test
    void testLeftOfDiagonalGoal() {
        setGoal(144, 144);

        double angle = Shooter.getTurretAngle(120, 0, Math.PI / 2);
        assertTrue(angle > 0);
    }

    @Test
    void testBehindGoal_diagonalGoal() {
        setGoal(144, 144);

        double angle = Shooter.getTurretAngle(144, 180, -Math.PI / 2);
        assertEquals(Math.PI, Math.abs(angle), 1e-1);
    }

    @Test
    void testDiagonalApproach_diagonalGoal() {
        setGoal(144, 144);

        double angle = Shooter.getTurretAngle(72, 72, Math.PI / 4);
        assertTrue(angle > 0);
    }
}
