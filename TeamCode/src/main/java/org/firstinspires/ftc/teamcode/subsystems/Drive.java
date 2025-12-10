package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import java.util.Arrays;
import java.util.Collections;

public class Drive extends SubsystemBase {
    private final Follower follower;

    private double movementSpeed = 1;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Command goToBase() {
        // Use DeferredCommand to create the path and command on schedule
        return new DeferredCommand(
                () -> {
                    PathChain parking = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            new Pose(38.5, 33.5)
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    getClosestRightAngle(follower.getHeading())
                            )
                            .setBrakingStrength(0.5)
                            .build();

                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, parking),
                            new InstantCommand(follower::startTeleopDrive)
                    );
                },
                Collections.singletonList(this)
        );
    }

    public Command goToCenter() {
        // Use DeferredCommand to create the path and command on schedule
        return new DeferredCommand(
                () -> {
                    PathChain parking = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            new Pose(72, 72)
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    Math.toRadians(0)
                            )
                            .setBrakingStrength(0.5)
                            .build();

                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, parking),
                            new InstantCommand(follower::startTeleopDrive)
                    );
                },
                Collections.singletonList(this)
        );
    }

    public void joystickDrive(Gamepad gamepad) {
        follower.setTeleOpDrive(
                -gamepad.left_stick_y * movementSpeed,
                -gamepad.left_stick_x * movementSpeed,
                -gamepad.right_stick_x * movementSpeed,
                true
        );
    }

    public double getMovementSpeed() {
        return movementSpeed;
    }

    public void setMovementSpeed(double speed) {
        movementSpeed = speed;
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
