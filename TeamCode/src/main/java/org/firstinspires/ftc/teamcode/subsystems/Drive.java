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
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.teamcode.enums.Alliance;

import java.util.Collections;

public class Drive extends SubsystemBase {
    private final Follower follower;
    private final Alliance alliance;
    private final boolean isRobotCentric;
    private final boolean tabletopMode;
    private final boolean debugMode;

    private boolean shootingMode;
    private boolean isHoldingPosition = false;

    public Drive(Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
        this.isRobotCentric = Settings.get("robot_centric", true);
        this.tabletopMode = Settings.get("tabletop_mode", false);
        this.debugMode = Settings.get("debug_mode", false);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Command goToGate() {
        if (tabletopMode) return new InstantCommand();

        return new DeferredCommand(
                () -> {
                    PathChain openGate = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            getRelative(new Pose(132, 60.5))
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    getRelative(Math.toRadians(45))

                            )
                            .setTranslationalConstraint(1)
                            .setBrakingStrength(1)
                            .build();

                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, openGate),
                            new WaitCommand(1000),
                            new InstantCommand(() -> follower.startTeleopDrive(USE_BRAKE_MODE))
                    );
                },
                Collections.singletonList(this)
        );
    }

    public Command goToBase() {
        if (tabletopMode) return new InstantCommand();

        // Use DeferredCommand to create the path and command on schedule
        return new DeferredCommand(
                () -> {
                    PathChain parking = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            getRelative(new Pose(34, 29))
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    follower.getHeading(),
                                    getRelative(Math.toRadians(45))
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
        if (tabletopMode || !debugMode) return new InstantCommand();

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

    public void teleOpDrive(Gamepad gamepad) {
        if (tabletopMode) return;

        double leftY = -gamepad.left_stick_y;
        double leftX = -gamepad.left_stick_x;
        double rightX = -gamepad.right_stick_x;

        boolean hasInput = Math.abs(leftY) > 0.1 || Math.abs(leftX) > 0.1 || Math.abs(rightX) > 0.1;

        if (hasInput) {
            // RISING EDGE: We were holding, now we are moving. Switch back to TeleOp control.
            if (isHoldingPosition) {
                follower.startTeleopDrive(USE_BRAKE_MODE);
                isHoldingPosition = false;
            }

            double headingOffset = (alliance == Alliance.BLUE && !isRobotCentric) ? Math.PI : 0;

            follower.setTeleOpDrive(
                    leftY * (shootingMode ? SHOOTING_FORWARD_SPEED : FORWARD_SPEED),
                    leftX * (shootingMode ? SHOOTING_STRAFE_SPEED : STRAFE_SPEED),
                    rightX * (shootingMode ? SHOOTING_TURN_SPEED : TURN_SPEED),
                    isRobotCentric,
                    headingOffset
            );
        } else {
            // Only lock position if we aren't already holding AND velocity is low enough
            if (!isHoldingPosition) {
                double currentVelocity = follower.getVelocity().getMagnitude();

                if (currentVelocity < HOLD_VELOCITY_THRESHOLD) {
                    follower.holdPoint(follower.getPose());
                    isHoldingPosition = true;
                } else {
                    follower.setTeleOpDrive(0, 0, 0, isRobotCentric);
                }
            }
        }
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

    private double getGateAngle(double heading) {
        // 1. Normalize heading to [0, 2π)
        double normalizedHeading = ((heading % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);

        // 2. If heading is between 0 and π (0 to 180°), π/2 is the closest vertical angle
        if (normalizedHeading > 0 && normalizedHeading < Math.PI) {
            return Math.PI / 2;
        }

        // 3. Otherwise (between 180 and 360°), 3π/2 is the closest vertical angle
        return 3 * Math.PI / 2;
    }

    private Pose getRelative(Pose originalPose) {
        if (alliance == Alliance.BLUE) {
            return originalPose.mirror();
        }

        return originalPose;
    }

    private double getRelative(double headingRad) {
        if (alliance == Alliance.BLUE) {
            return MathFunctions.normalizeAngle(Math.PI - headingRad);
        }

        return headingRad;
    }
}
