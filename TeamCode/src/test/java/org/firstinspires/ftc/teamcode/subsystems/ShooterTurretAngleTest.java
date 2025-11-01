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
    void testFacingStraightAtGoal_originGoal() {
        setGoal(0, 144);
        assertEquals(0.0, Shooter.getTurretAngle(0, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRobotToLeftOfGoal_originGoal() {
        setGoal(0, 144);
        assertEquals(2 * Math.PI - 0.165, Shooter.getTurretAngle(-24, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRobotRightOfGoal_originGoal() {
        setGoal(0, 144);
        assertEquals(0.165, Shooter.getTurretAngle(24, 0, Math.PI / 2), EPS);
    }

    @Test
    void testFacingEast_originGoal() {
        setGoal(0, 144);
        assertEquals(Math.PI / 2, Shooter.getTurretAngle(0, 0, 0), EPS);
    }

    @Test
    void testFacingSouth_originGoal() {
        setGoal(0, 144);
        assertEquals(Math.PI, Shooter.getTurretAngle(0, 0, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testFacingGoal_diagonalGoal() {
        setGoal(144, 144);
        assertEquals(0.0, Shooter.getTurretAngle(144, 0, Math.PI / 2), EPS);
    }

    @Test
    void testLeftOfDiagonalGoal() {
        setGoal(144, 144);
        assertEquals(2 * Math.PI - 0.165, Shooter.getTurretAngle(120, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRightOfDiagonalGoal() {
        setGoal(144, 144);
        assertEquals(0.165, Shooter.getTurretAngle(168, 0, Math.PI / 2), EPS);
    }

    @Test
    void testFacingSouth_diagonalGoal() {
        setGoal(144, 144);
        assertEquals(Math.PI, Shooter.getTurretAngle(144, 0, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testBehindGoal_diagonalGoal() {
        setGoal(144, 144);
        assertEquals(0, Shooter.getTurretAngle(144, 180, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testGoalBelowRobot() {
        setGoal(0, -144);
        assertEquals(0.0, Shooter.getTurretAngle(0, 0, 3 * Math.PI / 2), EPS);
    }

    @Test
    void testGoalBelowFacingUp() {
        setGoal(0, -144);
        assertEquals(Math.PI, Shooter.getTurretAngle(0, 0, Math.PI / 2), EPS);
    }

    @Test
    void testRobotAtGoalPosition() {
        setGoal(0, 144);
    }

    @Test
    void testFarDiagonalGoal() {
        setGoal(300, 300);
        assertEquals(7 * Math.PI / 4, Shooter.getTurretAngle(0, 0, Math.PI / 2), EPS);
    }
}
