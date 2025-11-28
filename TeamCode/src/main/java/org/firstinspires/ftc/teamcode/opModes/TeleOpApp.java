package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_POSSIBLE_MIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

@TeleOp
public class TeleOpApp extends ComplexOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setPose(new Pose(72, 72));

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, follower.poseTracker);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.collect()))
                .whenReleased(new InstantCommand(() -> intake.stop()));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.release()))
                .whenReleased(new InstantCommand(() -> intake.stop()));

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
//                .whenPressed(new InstantCommand(() -> {
//                    Command cmd = drive.goToBase();
//                    drive.setCurrentCommand(cmd);
//                    CommandScheduler.getInstance().schedule(cmd);
//                }));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new InstantCommand(() -> transfer.toggleTransfer(true)))
                .whenReleased(new InstantCommand(() -> transfer.toggleTransfer(false)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new ShootCommand(3, shooter, intake, transfer));
    }

    @Override
    public void run() {
        drive.joystickDrive(gamepad1);

//        shooter.updateHorizontalAngle();
//        shooter.updateVerticalAngle();

        // Immediately cancel drive command if joysticks are moved
        if (gamepadEx1.getLeftX() != 0 || gamepadEx1.getLeftY() != 0 || gamepadEx1.getRightX() != 0 || gamepadEx1.getRightY() != 0) {
            CommandScheduler.getInstance().cancel(drive.getCurrentCommand());
        }

        if (gamepad1.dpad_up) {
            shooter.setRawHoodPosition(MathFunctions.clamp(shooter.getRawHoodPosition() + 0.05, HOOD_POSSIBLE_MIN, 1));
        }

        if (gamepad1.dpad_down) {
            shooter.setRawHoodPosition(MathFunctions.clamp(shooter.getRawHoodPosition() - 0.05, HOOD_POSSIBLE_MIN, 1));
        }

        telemetry.addData("Robot x", follower.getPose().getX());
        telemetry.addData("Robot y", follower.getPose().getY());
        telemetry.addData("Robot heading", follower.getPose().getHeading());
        telemetry.addData("Turret Target", Shooter.calculateTurretAngle(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.addData("Intake RPM", intake.getRPM());
        telemetry.addData("PodX ticks", follower.getPoseTracker().getLocalizer().getLateralMultiplier());
        telemetry.addData("PodY ticks", follower.getPoseTracker().getLocalizer().getForwardMultiplier());
        telemetry.update();

        double inchesToMeters = 39.37;
        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Shooter/Turret Target", Shooter.calculateTurretAngle(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
    }
}
