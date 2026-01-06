package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.enums.Alliance;

import java.util.Collections;

public class Drive extends SubsystemBase {
    private final Follower follower;
    private final Alliance alliance;

    private boolean shootingMode;

    public Drive(Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Command goToGate(){
        return new DeferredCommand(
                () -> {
                    PathChain openGate = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            getRelative(new Pose(18.039, 70.911))
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    gateAngle(follower.getHeading())

                            )
                            .setTranslationalConstraint(1)
                            .setBrakingStrength(1)
                            .build();

                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, openGate)
                    );
                },
                Collections.singletonList(this)
        );
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
                                            getRelative(new Pose(38.5, 33.5))
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    getClosestRightAngle(follower.getHeading())

                            )
                            .setTranslationalConstraint(1)
                            .setBrakingStrength(1)
                            .build();

                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, parking),
                            new WaitCommand(1000),
                            new InstantCommand(() -> follower.startTeleopDrive(USE_BRAKE_MODE))
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
                            .setTranslationalConstraint(1)
                            .setBrakingStrength(1)
                            .build();

                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, parking),
                            new WaitCommand(1000),
                            new InstantCommand(() -> follower.startTeleopDrive(USE_BRAKE_MODE))
                    );
                },
                Collections.singletonList(this)
        );
    }

    public void joystickDrive(Gamepad gamepad) {
        follower.setTeleOpDrive(
                -gamepad.left_stick_y * (shootingMode ? SHOOTING_FORWARD_SPEED : FORWARD_SPEED),
                -gamepad.left_stick_x * (shootingMode ? SHOOTING_STRAFE_SPEED : STRAFE_SPEED),
                -gamepad.right_stick_x * (shootingMode ? SHOOTING_TURN_SPEED : TURN_SPEED),
                true
        );
    }

    public boolean getShootingMode() {
        return shootingMode;
    }

    public void setShootingMode(boolean enabled) {
        shootingMode = enabled;
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

    private double gateAngle(double heading){
        if(getClosestRightAngle(heading) >= 0 && getClosestRightAngle(heading) < 180){
            return Math.PI / 2;
        }
        return 3 * Math.PI / 2;

    }

    private Pose getRelative(Pose originalPose) {
        if (alliance == Alliance.RED) {
            return originalPose.mirror();
        }

        return originalPose;
    }

    private double getRelative(double headingRad) {
        if (alliance == Alliance.RED) {
            return MathFunctions.normalizeAngle(Math.PI - headingRad);
        }

        return headingRad;
    }
}
