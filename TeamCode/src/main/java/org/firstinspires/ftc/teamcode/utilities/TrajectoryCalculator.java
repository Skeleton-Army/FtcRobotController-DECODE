package org.firstinspires.ftc.teamcode.utilities;

import org.psilynx.psikit.core.wpi.math.Translation3d;

public class TrajectoryCalculator {

    // --- Physics Constants ---
    /** Gravity in meters/second^2 */
    private static final double G = 9.81;
    /** Air drag coefficient (matched to MATLAB script) */
    private static final double MU = 0.05909198612;

    /**
     * Generates a 3D ballistic trajectory using Runge-Kutta 4 integration
     * to account for air resistance (drag).
     *
     * @param startPos      The starting position of the projectile in Field-Centric coordinates.
     * <strong>Unit: Meters</strong> (x, y, z height of shooter).
     * @param v0            The initial muzzle velocity of the projectile.
     * <strong>Unit: Meters per second</strong>.
     * @param hoodAngle     The vertical pitch of the shooter (Theta).
     * <strong>Unit: Radians</strong>.
     * (0 = Horizontal, PI/2 = Vertical Up).
     * @param turretAngle   The horizontal yaw of the shooter (Phi).
     * <strong>Unit: Radians</strong>.
     * (Standard field-centric angle, usually 0 is +X).
     * @param totalTime     The total duration to simulate.
     * <strong>Unit: Seconds</strong>.
     * @param dt            The time step for the integration (resolution).
     * <strong>Unit: Seconds</strong> (Recommended: 0.01 to 0.05).
     * @return              An array of Translation3d points representing the path.
     */
    public static Translation3d[] generateTrajectory(
            Translation3d startPos,
            double v0,
            double hoodAngle,
            double turretAngle,
            double totalTime,
            double dt
    ) {
        int steps = (int) Math.ceil(totalTime / dt);
        Translation3d[] trajectory = new Translation3d[steps];

        // 1. Decompose Angles
        // Hood Angle (Pitch) controls Vertical (Z) vs Horizontal (XY) magnitude
        double vZ_init = v0 * Math.sin(hoodAngle);
        double vXY_mag = v0 * Math.cos(hoodAngle);

        // Turret Angle (Yaw) splits the Horizontal magnitude into X and Y
        double vX_init = vXY_mag * Math.cos(turretAngle);
        double vY_init = vXY_mag * Math.sin(turretAngle);

        // 2. Initialize State Vector
        // State: [x, y, z, vx, vy, vz]
        // All positions in Meters, Velocities in m/s
        double[] state = new double[]{
                startPos.getX(),
                startPos.getY(),
                startPos.getZ(),
                vX_init,
                vY_init,
                vZ_init
        };

        // 3. Integration Loop (RK4)
        for (int i = 0; i < steps; i++) {
            // Store the current position in the trajectory array
            trajectory[i] = new Translation3d(state[0], state[1], state[2]);

            // Calculate the state for the next time step
            state = step_rk4(state, dt);

            // Optional optimization: Stop calculation if it hits the ground
            if (state[2] < 0) {
                // You might want to handle resizing the array or breaking here
                // to avoid drawing points underground.
            }
        }

        return trajectory;
    }

    /**
     * Performs a single Runge-Kutta 4 (RK4) integration step.
     * This approximates the state at time t + h based on the state at time t.
     *
     * @param state The current state vector [x, y, z, vx, vy, vz].
     * @param h     The time step (delta time).
     * @return      The new state vector after time h.
     */
    private static double[] step_rk4(double[] state, double h) {
        // k1: Slopes at the beginning of the interval
        double[] k1 = computeDerivatives(state);

        // k2: Slopes at the midpoint (using k1)
        double[] stateK2 = addVectors(state, scaleVector(k1, 0.5 * h));
        double[] k2 = computeDerivatives(stateK2);

        // k3: Slopes at the midpoint (using k2)
        double[] stateK3 = addVectors(state, scaleVector(k2, 0.5 * h));
        double[] k3 = computeDerivatives(stateK3);

        // k4: Slopes at the end of the interval (using k3)
        double[] stateK4 = addVectors(state, scaleVector(k3, h));
        double[] k4 = computeDerivatives(stateK4);

        // Combine slopes: state + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
        double[] sumK = new double[6];
        for (int i = 0; i < 6; i++) {
            sumK[i] = k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i];
        }

        return addVectors(state, scaleVector(sumK, h / 6.0));
    }

    /**
     * Computes the derivatives (velocities and accelerations) for the current state.
     * This encapsulates the physics model (Gravity + Drag).
     *
     * @param s The current state vector [x, y, z, vx, vy, vz].
     * @return  The derivative vector [vx, vy, vz, ax, ay, az].
     */
    private static double[] computeDerivatives(double[] s) {
        // Extract velocities
        double vx = s[3];
        double vy = s[4];
        double vz = s[5];

        // Calculate total velocity magnitude (speed)
        double v = Math.sqrt(vx * vx + vy * vy + vz * vz);

        // --- Physics Engine ---
        // 1. Position Derivative is simply Velocity
        double dx = vx;
        double dy = vy;
        double dz = vz;

        // 2. Velocity Derivative is Acceleration (Forces / Mass)
        // Drag Force = -mu * v * velocity_vector
        // Gravity = -9.81 on Z axis

        double ax = -MU * vx * v;
        double ay = -MU * vy * v;
        double az = -G - (MU * vz * v);

        return new double[]{dx, dy, dz, ax, ay, az};
    }

    // --- Helper Math Functions ---

    private static double[] addVectors(double[] a, double[] b) {
        double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = a[i] + b[i];
        }
        return result;
    }

    private static double[] scaleVector(double[] a, double scalar) {
        double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = a[i] * scalar;
        }
        return result;
    }
}