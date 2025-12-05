package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

public class Drive extends SubsystemBase {
    private final Follower follower;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Command goToBase() {
        PathChain parking = follower
                .pathBuilder()
                .addPath(new BezierLine(follower.getPose(), new Pose(38.5, 33.5)))
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        getClosestRightAngle(follower.getHeading())
                )
                .build();

        return new FollowPathCommand(follower, parking);
    }

    public void joystickDrive(Gamepad gamepad) {
        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
    }

    /**
     * Returns the closest right angle to the given heading in radians.
     */
    private double getClosestRightAngle(double heading) {
        heading = ((heading % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI); // Normalize to [0, 2π]

        double[] rightAngles = { 0, Math.PI / 2, Math.PI, 3 * Math.PI / 2 };
        double closest = 0;
        double minDiff = Double.MAX_VALUE;

        for (double angle : rightAngles) {
            double diff = Math.abs(heading - angle);
            diff = Math.min(diff, 2 * Math.PI - diff); // account for wraparound (e.g. near 2π)

            if (diff < minDiff) {
                minDiff = diff;
                closest = angle;
            }
        }

        return closest;
    }
}
