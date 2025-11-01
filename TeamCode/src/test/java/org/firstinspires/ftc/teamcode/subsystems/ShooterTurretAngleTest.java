package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.junit.jupiter.api.Test;

public class ShooterTurretAngleTest {

    private static final double EPS = 1e-3;

    private void setGoal(double x, double y) {
        ShooterConfig.GOAL_X = x;
        ShooterConfig.GOAL_Y = y;
        ShooterConfig.DISTANCE_TO_BOT_CENTER = 0;
    }

    @Test
    public void testAngleWrapping() {
        assertEquals(50, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(0), Math.toRadians(50), -2 * Math.PI, 2 * Math.PI)), EPS);
        assertEquals(60, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(40), Math.toRadians(60), -2 * Math.PI, 2 * Math.PI)), EPS);
        assertEquals(1, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(359), Math.toRadians(1), -2 * Math.PI, 2 * Math.PI)), EPS);
        assertEquals(-60, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(60), Math.toRadians(300), -2 * Math.PI, 2 * Math.PI)), EPS);
        assertEquals(-1, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(-359), Math.toRadians(359), -2 * Math.PI, 2 * Math.PI)), EPS);

        assertEquals(179, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(-179), Math.toRadians(179), -Math.PI, Math.PI)), EPS);
        assertEquals(1, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(-1), Math.toRadians(1), -Math.PI, Math.PI)), EPS);
        assertEquals(-179, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(179), Math.toRadians(181), -Math.PI, Math.PI)), EPS);
        assertEquals(180, Math.toDegrees(Shooter.wrapToTarget(Math.toRadians(180), Math.toRadians(180), -Math.PI, Math.PI)), EPS);
    }

    @Test
    void testFacingStraightAtGoal_originGoal() {
        setGoal(0, 144);
        assertEquals(0.0, Shooter.calculateTurretAngle(0, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRobotToLeftOfGoal_originGoal() {
        setGoal(0, 144);
        assertEquals(2 * Math.PI - 0.165, Shooter.calculateTurretAngle(-24, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRobotRightOfGoal_originGoal() {
        setGoal(0, 144);
        assertEquals(0.165, Shooter.calculateTurretAngle(24, 0, Math.PI / 2), EPS);
    }

    @Test
    void testFacingEast_originGoal() {
        setGoal(0, 144);
        assertEquals(Math.PI / 2, Shooter.calculateTurretAngle(0, 0, 0), EPS);
    }

    @Test
    void testFacingSouth_originGoal() {
        setGoal(0, 144);
        assertEquals(Math.PI, Shooter.calculateTurretAngle(0, 0, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testFacingGoal_diagonalGoal() {
        setGoal(144, 144);
        assertEquals(0.0, Shooter.calculateTurretAngle(144, 0, Math.PI / 2), EPS);
    }

    @Test
    void testLeftOfDiagonalGoal() {
        setGoal(144, 144);
        assertEquals(2 * Math.PI - 0.165, Shooter.calculateTurretAngle(120, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRightOfDiagonalGoal() {
        setGoal(144, 144);
        assertEquals(0.165, Shooter.calculateTurretAngle(168, 0, Math.PI / 2), EPS);
    }

    @Test
    void testFacingSouth_diagonalGoal() {
        setGoal(144, 144);
        assertEquals(Math.PI, Shooter.calculateTurretAngle(144, 0, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testBehindGoal_diagonalGoal() {
        setGoal(144, 144);
        assertEquals(0, Shooter.calculateTurretAngle(144, 180, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testGoalBelowRobot() {
        setGoal(0, -144);
        assertEquals(0.0, Shooter.calculateTurretAngle(0, 0, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testGoalBelowFacingUp() {
        setGoal(0, -144);
        assertEquals(Math.PI, Shooter.calculateTurretAngle(0, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRobotAtGoalPosition() {
        setGoal(0, 144);
    }

    @Test
    void testFarDiagonalGoal() {
        setGoal(300, 300);
        assertEquals(7 * Math.PI / 4, Shooter.calculateTurretAngle(0, 0, Math.PI / 2), EPS);
    }
}
