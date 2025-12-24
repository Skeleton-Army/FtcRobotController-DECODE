package org.firstinspires.ftc.teamcode.utilities;

import org.psilynx.psikit.core.wpi.math.Translation3d;

/*
    This class will debug all the necessary stuff (such as shooter subsystem, robot pose ect)
       It will also consist all the relevant tools/functions
 */

public class Debugger {

    // generates the ball path to the goal
    public static Translation3d[] generateTrajectory(
            Translation3d startPos,
            double v0,
            double theta,
            double phi,
            double totalTime,
            double dt
    ) {
        int steps = (int) Math.ceil(totalTime / dt);
        Translation3d[] trajectory = new Translation3d[steps];

        final double g = 9.81;

        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);
        double cosP = Math.cos(phi);
        double sinP = Math.sin(phi);

        double vX = v0 * cosT * cosP;
        double vY = v0 * cosT * sinP;
        double vZ = v0 * sinT;

        for (int i = 0; i < steps; i++) {
            double t = i * dt;

            double x = startPos.getX() + vX * t;
            double y = startPos.getY() + vY * t;
            double z = startPos.getZ() + vZ * t - 0.5 * g * t * t;

            trajectory[i] = new Translation3d(x, y, z);
        }

        return trajectory;
    }

}
