package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.*;
import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.X_OFFSET;
import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.Y_OFFSET;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
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
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.opModes.TeleOpApp;

import java.util.Collections;

public class Drive extends SubsystemBase {
    public static final double ROBOT_WIDTH = 16.53; // Side-to-side
    public static final double ROBOT_LENGTH = 14.96; // Front-to-back

    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(188, 188), new Point(94, 94), new Point(0, 188));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(72, 0), new Point(72, 46), new Point(94, 68), new Point(117,46), new Point(117, 0));

    private final PolygonZone robotZone = new PolygonZone(ROBOT_LENGTH, ROBOT_WIDTH);
    private final PolygonZone futureRobotZone = new PolygonZone(ROBOT_LENGTH, ROBOT_WIDTH);

    private final Follower follower;
    private final Alliance alliance;
    private final boolean isRobotCentric;
    private final boolean tabletopMode;
    private final boolean debugMode;

    private boolean shootingMode;
    private boolean isHoldingPosition = false;

    private boolean enabled = true;

    private long lastUpdateFrame = -1;
    private long lastZoneFrame = -1;
    private long lastPredictiveFrame = -1;
    private boolean cachedIsInside;
    private boolean cachedIsInsidePredictive;
    private long lastCloseDistanceFrame = -1;
    private long lastFarDistanceFrame = -1;
    private double cachedCloseDistance;
    private double cachedFarDistance;

    public Drive(Follower follower, Alliance alliance) {
        this.follower = follower;
        this.alliance = alliance;
        this.isRobotCentric = Settings.get("robot_centric", true);
        this.tabletopMode = Settings.get("tabletop_mode", false);
        this.debugMode = Settings.get("debug_mode", false);
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        follower.update();

        lastUpdateFrame++;

        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());
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
                                            getRelative(new Pose(181, 107))
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
                                            getRelative(new Pose(125.5, 155.5))
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
                                            new Pose(X_OFFSET, Y_OFFSET)
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

    public Command LoadingZoneCycle() {
        if (tabletopMode) return new InstantCommand();


        Pose loadingZoneEnd = new Pose(22.5,38.5, Math.toRadians(160)); // pose for intaking at the opposing loading zone, the end of the path
        if (follower.getPose().getX() < GoalPositions.HALF_FIELD_LENGTH)
            loadingZoneEnd.mirror();

        return new DeferredCommand(
                () -> {
                    PathChain path = follower
                            .pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), loadingZoneEnd))
                            .setHeadingInterpolation(HeadingInterpolator.linear(follower.getHeading(), loadingZoneEnd.getHeading()))
                            .build();


                    return new SequentialCommandGroup(
                            new FollowPathCommand(follower, path),
                            new WaitCommand(1000),
                            new InstantCommand(() -> follower.startTeleopDrive(USE_BRAKE_MODE))
                    );

                },
                Collections.singletonList(this)
                );
    }

    public void teleOpDrive(Gamepad gamepad) {
        if (tabletopMode || !enabled) return;

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

    public void holdPoint(Pose pose) {
        if (isHoldingPosition) {
            follower.holdPoint(pose);
        }
    }

    public boolean isInsideLaunchZone() {
        if (shouldUpdateCache(lastZoneFrame)) {
            cachedIsInside = robotZone.isInside(closeLaunchZone) || robotZone.isInside(farLaunchZone);
            lastZoneFrame = lastUpdateFrame;
        }
        return cachedIsInside;
    }

    public boolean isInsideLaunchZonePredictive() {
        if (tabletopMode) return true;

        if (shouldUpdateCache(lastPredictiveFrame)) {
            final double PREDICTION_TIME = 0.3;
            double futureX = follower.getPose().getX() + (follower.getVelocity().getXComponent() * PREDICTION_TIME);
            double futureY = follower.getPose().getY() + (follower.getVelocity().getYComponent() * PREDICTION_TIME);

            futureRobotZone.setPosition(futureX, futureY);
            futureRobotZone.setRotation(follower.getPose().getHeading());


            cachedIsInsidePredictive = futureRobotZone.isInside(closeLaunchZone) || futureRobotZone.isInside(farLaunchZone);
            lastPredictiveFrame = lastUpdateFrame;
        }
        return cachedIsInsidePredictive;
    }

    public double distanceFromLaunchZone() {
        return Math.min(distanceFromCloseLaunchZone(), distanceFromFarLaunchZone());
    }

    public double distanceFromCloseLaunchZone() {
        if (shouldUpdateCache(lastCloseDistanceFrame)) {
            cachedCloseDistance = robotZone.distanceTo(closeLaunchZone);
            lastCloseDistanceFrame = lastUpdateFrame;
        }
        return cachedCloseDistance;
    }

    public double distanceFromFarLaunchZone() {
        if (shouldUpdateCache(lastFarDistanceFrame)) {
            cachedFarDistance = robotZone.distanceTo(farLaunchZone);
            lastFarDistanceFrame = lastUpdateFrame;
        }
        return cachedFarDistance;
    }

    public void disable() {
        this.enabled = false;
        follower.setTeleOpDrive(0, 0, 0);
        follower.breakFollowing();
        follower.setMaxPower(0);
    }

    public void enable() {
        this.enabled = true;
        follower.setMaxPower(1);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean getShootingMode() {
        return shootingMode;
    }

    public void setShootingMode(boolean enabled) {
        shootingMode = enabled;
    }

    private boolean shouldUpdateCache(long frameToken) {
        return lastUpdateFrame != frameToken;
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
            return originalPose.mirror(GoalPositions.FIELD_LENGTH);
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
