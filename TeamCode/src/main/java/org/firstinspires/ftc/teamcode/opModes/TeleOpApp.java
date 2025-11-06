package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

@TeleOp
public class TeleOpApp extends ComplexOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private PathChain parking;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, follower.poseTracker);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // TODO: Change according to red and blue alliance
        parking = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                new Pose(38.5,33.5)
                        )
                )
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(getClosestRightAngle(follower)))
                .build();

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.collect()))
                .whenReleased(new InstantCommand(() -> intake.stop()));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.release()))
                .whenReleased(new InstantCommand(() -> intake.stop()));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(new FollowPathCommand(follower, parking));
    }

    @Override
    public void run() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        shooter.updateHorizontalAngle();

        telemetry.addData("Robot x", follower.getPose().getX());
        telemetry.addData("Robot y", follower.getPose().getY());
        telemetry.addData("Robot heading", follower.getPose().getHeading());
        telemetry.addData("Turret Target", Shooter.calculateTurretAngle(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));

        telemetry.update();

        double inchesToMeters = 39.37;
        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("Robot Pose", robotPose);
    }

    public int getClosestRightAngle(Follower follower) {
        double heading = follower.getHeading();
        heading = ((heading % 360) + 360) % 360; // normalize to 0–360

        int closest = 0;
        double minDiff = 360;

        int[] rightAngles = {0, 90, 180, 270};
        for (int angle : rightAngles) {
            double diff = Math.abs(heading - angle);
            diff = Math.min(diff, 360 - diff); // handle wrap-around (e.g. 359° to 0°)
            if (diff < minDiff) {
                minDiff = diff;
                closest = angle;
            }
        }

        return closest;
    }
}
