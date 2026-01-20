package org.firstinspires.ftc.teamcode.consts;

import org.firstinspires.ftc.teamcode.calculators.HoodSolution;

import java.sql.Array;

public class ShooterLookupTable {
    public static final double MIN_DIST = 0.7;
    public static final double MAX_DIST = 4.3;
    public static final double MIN_VEL = 4.4;
    public static final double MAX_VEL = 8.0;
    public static final int DIST_STEPS = 80;
    public static final int VEL_STEPS = 160;
    public static double[][] ANGLE_TABLE = new double[DIST_STEPS][VEL_STEPS];

    public static boolean[][] VALIDITY_TABLE = new boolean[DIST_STEPS][VEL_STEPS];


    public static double[] VELOCITY_ARRAY;
    public static HoodSolution lookup(double distance, double velocity) {
        distance = Math.max(MIN_DIST, Math.min(MAX_DIST, distance));
        velocity = Math.max(MIN_VEL, Math.min(MAX_VEL, velocity));

        double dIdx = (distance - MIN_DIST) / (MAX_DIST - MIN_DIST) * (DIST_STEPS - 1);
        double vIdx = (velocity - MIN_VEL) / (MAX_VEL - MIN_VEL) * (VEL_STEPS - 1);

        int r0 = (int) dIdx;
        int r1 = Math.min(r0 + 1, DIST_STEPS - 1);
        int c0 = (int) vIdx;
        int c1 = Math.min(c0 + 1, VEL_STEPS - 1);

        double dr = dIdx - r0;
        double dc = vIdx - c0;

        double v00 = ANGLE_TABLE[r0][c0];
        double v01 = ANGLE_TABLE[r0][c1];
        double v10 = ANGLE_TABLE[r1][c0];
        double v11 = ANGLE_TABLE[r1][c1];

        double interpolatedAngle = v00 * (1 - dr) * (1 - dc) +
                v01 * (1 - dr) * dc +
                v10 * dr * (1 - dc) +
                v11 * dr * dc;

        boolean valid = VALIDITY_TABLE[r0][c0] && VALIDITY_TABLE[r1][c1];

        return new HoodSolution(interpolatedAngle, valid);
    }
}
