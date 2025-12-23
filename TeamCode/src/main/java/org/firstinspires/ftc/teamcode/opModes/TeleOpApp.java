package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.settings.Settings;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.calculators.TwoZonesCalculator;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

@TeleOp
public class TeleOpApp extends ComplexOpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(17, 17);

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;

    private VoltageSensor voltageSensor;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private boolean debugMode;
    private boolean tabletopMode;
    private Alliance alliance;

    final double inchesToMeters = 39.37;

    @Override
    public void initialize() {
        debugMode = Settings.get("debug_mode", false);
        tabletopMode = Settings.get("tabletop_mode", false);
        alliance = Settings.get("alliance", Alliance.RED);
        Pose startPose = new Pose(72, 72);
        if (!debugMode) startPose = Settings.get("pose", new Pose(72, 72));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setPose(startPose);
        follower.setMaxPower(1);

        IShooterCalculator shooterCalc = new TwoZonesCalculator(ShooterCoefficients.CLOSE_HOOD_COEFFS, ShooterCoefficients.FAR_HOOD_COEFFS, ShooterCoefficients.VEL_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new InstantCommand(() -> {
                            intake.collect();
                            transfer.toggleTransfer(true, true);
                        }, intake, transfer)
                )
                .whenReleased(new InstantCommand(() -> {
                            intake.stop();
                            transfer.toggleTransfer(false);
                        }, intake, transfer)
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    intake.release();
                    transfer.toggleTransfer(true, true);
                    }, intake, transfer)
                )
                .whenReleased(new InstantCommand(() -> {
                            intake.stop();
                            transfer.toggleTransfer(false);
                        }, intake, transfer)
                );

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
//                .whenPressed(new InstantCommand(() -> transfer.toggleTransfer(true)))
//                .whenReleased(new InstantCommand(() -> transfer.toggleTransfer(false)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .and(new Trigger(this::isInsideLaunchZone))
                .whenActive(new ShootCommand(shooter, intake, transfer, drive));

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .and(new Trigger(this::isInsideLaunchZone))
                .whileActiveContinuous(new ShootCommand(shooter, intake, transfer, drive));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getVerticalManualMode()) shooter.setHoodPosition(shooter.getRawHoodPosition() + 0.01);
                            else shooter.setVerticalOffset(shooter.getVerticalOffset() + 0.05);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getVerticalManualMode()) shooter.setHoodPosition(shooter.getRawHoodPosition() - 0.01);
                            else shooter.setVerticalOffset(shooter.getVerticalOffset() - 0.05);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getHorizontalManualMode()) shooter.setHorizontalAngle(shooter.getTurretAngle(AngleUnit.RADIANS) + 0.2);
                            else shooter.setHorizontalOffset(shooter.getHorizontalOffset() + 0.01);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getHorizontalManualMode()) shooter.setHorizontalAngle(shooter.getTurretAngle(AngleUnit.RADIANS) - 0.2);
                            else shooter.setHorizontalOffset(shooter.getHorizontalOffset() - 0.01);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .toggleWhenPressed(
                        new InstantCommand(() -> {
                            shooter.setHorizontalManualMode(true);
                            shooter.setVerticalManualMode(true);
                        }),
                        new InstantCommand(() -> {
                            shooter.setHorizontalManualMode(false);
                            shooter.setVerticalManualMode(false);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(
                        new InstantCommand(shooter::resetTurret)
                );

        if (!tabletopMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                    .whenPressed(drive.goToBase());
        }

        if (!tabletopMode && debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(drive.goToCenter());
        }

        if (!tabletopMode && !debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(this::resetPoseToNearestCorner);
        }
    }

    @Override
    public void run() {
        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());

        if (!tabletopMode) {
            drive.joystickDrive(gamepad1);
        }

        // Immediately cancel drive command if joysticks are moved
        if (drive.getCurrentCommand() != null && (gamepadEx1.getLeftX() != 0 || gamepadEx1.getLeftY() != 0 || gamepadEx1.getRightX() != 0 || gamepadEx1.getRightY() != 0)) {
            CommandScheduler.getInstance().cancel(drive.getCurrentCommand());
            follower.startTeleopDrive(USE_BRAKE_MODE);
        }

        // Cancel shooting if not in a launch zone
        if (!isInsideLaunchZone()) {
            CommandScheduler.getInstance().cancel(shooter.getCurrentCommand());
        }

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d robotPose = new Pose2d(-rotatedPose.getX() / inchesToMeters, -rotatedPose.getY() / inchesToMeters, new Rotation2d(rotatedPose.getHeading() - Math.PI));

        telemetry.addData("Pedro Robot x", 72 - follower.getPose().getX());
        telemetry.addData("Pedro Robot y", 72 - follower.getPose().getY());
        telemetry.addData("Pedro Robot heading", follower.getPose().getHeading());
        telemetry.addData("Robot x", robotPose.getX());
        telemetry.addData("Robot y", robotPose.getY());
        telemetry.addData("Robot heading", robotPose.getRotation().getDegrees());
        telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
        telemetry.addData("!Inside LAUNCH ZONE", isInsideLaunchZone());
        telemetry.addData("Current Voltage", voltageSensor.getVoltage());
        telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret error (deg)", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret solution error (deg)", shooter.solution.getHorizontalAngle() - shooter.getTurretAngle(AngleUnit.DEGREES));

        telemetry.addData("hood pos", shooter.getRawHoodPosition());
        telemetry.addData("hood angle(deg)", shooter.getHoodAngle());
        telemetry.addData("solution angle(deg)", Math.toDegrees(shooter.solution.getVerticalAngle()));
        telemetry.addData("distance from goal: ", follower.getPose().distanceFrom(GoalPositions.BLUE_GOAL) / inchesToMeters);
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.addData("Target RPM", shooter.solution.getVelocity());
        telemetry.addData("Flywheel error: ", Math.abs(shooter.getRPM() - shooter.solution.getVelocity()));
        telemetry.addData("Recovery Time", shooter.getRecoveryTime());
        telemetry.addData("calculating recovery", shooter.calculatedRecovery);
        telemetry.addData("!Reached RPM", shooter.reachedRPM());
        telemetry.addData("!Detected artifact", transfer.isArtifactDetected());

        telemetry.addData("Shot Hood Angle", shooter.shotHoodAngle);
        telemetry.addData("Shot Turret Angle", shooter.shotTurretAngle);
        telemetry.addData("Shot Flywheel RPM", shooter.shotFlywheelRPM);
        telemetry.addData("Shot goal distance", shooter.shotGoalDistance);
        telemetry.update();

        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Voltage", voltageSensor.getVoltage());
        Logger.recordOutput("Inside LAUNCH ZONE", isInsideLaunchZone());
        Logger.recordOutput("Reached RPM", shooter.reachedRPM());
        Logger.recordOutput("Reached Angle", shooter.reachedAngle());
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
        Logger.recordOutput("Shooter/Flywheel Error", Math.abs(shooter.getRPM() - shooter.solution.getVelocity()));
        Logger.recordOutput("Shooter/Flywheel Target", shooter.getTargetRPM());
        Logger.recordOutput("Shooter/Hood Raw Position", shooter.getRawHoodPosition());
        Logger.recordOutput("Shooter/Hood Angle (deg)", shooter.getHoodAngle());
        Logger.recordOutput("Turret/Turret Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        Logger.recordOutput("Turret/Turret Angle error (deg)", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));
    }

    @Override
    public void end() {
        Settings.set("pose", follower.getPose(), false);
    }

    public boolean isInsideLaunchZone() {
        boolean insideClose = robotZone.isInside(closeLaunchZone);
        boolean insideFar = robotZone.isInside(farLaunchZone);
        return insideClose || insideFar || debugMode;
    }

    private void resetPoseToNearestCorner() {
        Pose currentPose = follower.getPose();

        final double WIDTH = 17;
        final double HEIGHT = 17;
        final double X_OFFSET = WIDTH / 2.0;
        final double Y_OFFSET = HEIGHT / 2.0;

        Pose newPose;
        if (alliance == Alliance.RED) {
            newPose = new Pose(X_OFFSET, Y_OFFSET);
        } else {
            newPose = new Pose(144 - X_OFFSET, Y_OFFSET);
        }

        follower.setPose(new Pose(newPose.getX(), newPose.getY(), currentPose.getHeading()));
        follower.startTeleopDrive(USE_BRAKE_MODE);
    }
}
