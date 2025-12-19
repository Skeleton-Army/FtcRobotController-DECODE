package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.junit.jupiter.api.Test;

public class ShooterTurretAngleTest {

    private static final double EPS = 1e-3;

    @Test
    public void testAngleWrapping() {
        assertEquals(50, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(0), Math.toRadians(50), -2 * Math.PI, 2 * Math.PI, true)), EPS);
        assertEquals(60, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(40), Math.toRadians(60), -2 * Math.PI, 2 * Math.PI, true)), EPS);
        assertEquals(1, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(359), Math.toRadians(1), -2 * Math.PI, 2 * Math.PI, true)), EPS);
        assertEquals(-60, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(60), Math.toRadians(300), -2 * Math.PI, 2 * Math.PI, true)), EPS);
        assertEquals(-1, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(-359), Math.toRadians(359), -2 * Math.PI, 2 * Math.PI, true)), EPS);

        assertEquals(179, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(-179), Math.toRadians(179), -Math.PI, Math.PI, true)), EPS);
        assertEquals(1, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(-1), Math.toRadians(1), -Math.PI, Math.PI, true)), EPS);
        assertEquals(-179, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(179), Math.toRadians(181), -Math.PI, Math.PI, true)), EPS);
        assertEquals(180, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(180), Math.toRadians(180), -Math.PI, Math.PI, true)), EPS);
    }

    @Test
    public void testAngleNoWrapping() {
        assertEquals(50, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(0), Math.toRadians(50), -2 * Math.PI, 2 * Math.PI, false)), EPS);
        assertEquals(-360, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(0), Math.toRadians(-450), -2 * Math.PI, 2 * Math.PI, false)), EPS);
        assertEquals(360, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(0), Math.toRadians(450), -2 * Math.PI, 2 * Math.PI, false)), EPS);
        assertEquals(1, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(359), Math.toRadians(1), -2 * Math.PI, 2 * Math.PI, false)), EPS);
        assertEquals(179, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(-179), Math.toRadians(179), -Math.PI, Math.PI, false)), EPS);
        assertEquals(180, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(0), Math.toRadians(200), -Math.PI, Math.PI, false)), EPS);
        assertEquals(-180, Math.toDegrees(ShooterCalculator.wrapToTarget(Math.toRadians(0), Math.toRadians(-200), -Math.PI, Math.PI, false)), EPS);
    }
}
