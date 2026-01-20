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


    public static double[] VELOCITY_ARRAY = {4.400000, 4.400000, 4.400000, 4.400000, 4.400000, 4.400000, 4.563019, 4.608302, 4.707925, 4.789434, 4.889057, 4.988679, 5.110943, 5.228679, 5.346415, 5.464151, 5.600000, 5.740377, 5.876226, 6.012075, 6.288302, 6.446792, 6.587170, 6.759245, 6.913208, 7.284528, 7.438491, 7.393208, 7.510943, 7.520000, 7.529057, 7.538113, 7.547170, 7.556226, 7.569811, 7.578868, 7.587925, 7.596981, 7.610566, 7.619623, 7.628679, 7.633208, 7.646792, 7.655849, 7.664906, 7.673962, 7.683019, 7.692075, 7.701132, 7.714717, 7.719245, 7.728302, 7.737358, 7.746415, 7.760000, 7.764528, 7.773585, 7.782642, 7.796226, 7.800755, 7.809811, 7.823396, 7.832453, 7.836981, 7.846038, 7.859623, 7.864151, 7.873208, 7.886792, 7.895849, 7.900377, 7.913962, 7.918491, 7.927547, 7.941132, 7.945660, 7.954717, 7.963774, 7.968302, 7.981887};
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
