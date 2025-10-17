package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

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

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        intake = new Intake(hardwareMap);
        //shooter = new Shooter(hardwareMap, follower.poseTracker);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

//        new Trigger(() -> gamepad1.right_trigger > 0.1)
//                .whenActive(new ShootCommand(shooter));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.set(-1)))
                .whenReleased(new InstantCommand(() -> intake.set(0)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.set(1)))
                .whenReleased(new InstantCommand(() -> intake.set(0)));

        schedule(
                // TODO: Set shooter angle to GOAL
        );
    }

    @Override
    public void run() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        telemetry.addData("Robot x", follower.getPose().getX());
        telemetry.addData("Robot y", follower.getPose().getY());
        telemetry.addData("Robot heading", follower.getPose().getHeading());
        telemetry.update();

        double inchesToMeters = 39.37;
        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("Robot Pose", robotPose);
    }
}
