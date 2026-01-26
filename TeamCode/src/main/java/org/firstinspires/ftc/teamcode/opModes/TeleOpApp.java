package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.DISTANCE_THRESHOLD_METERS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.settings.Settings;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.calculators.LookupTableCalculator;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
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
    private Kickstand kickstand;

    private VoltageSensor voltageSensor;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private boolean debugMode;
    private boolean tabletopMode;
    private Alliance alliance;

    private TimerEx matchTime;

    public static final double INCHES_TO_METERS = 39.37;
    public static final double WIDTH = 15.74;
    public static final double HEIGHT = 16.53;
    public static final double X_OFFSET = WIDTH / 2.0;
    public static final double Y_OFFSET = HEIGHT / 2.0;

    @Override
    public void initialize() {
        matchTime = new TimerEx(120);

        debugMode = Settings.get("debug_mode", false);
        tabletopMode = Settings.get("tabletop_mode", false);
        alliance = Settings.get("alliance", Alliance.RED);
        Pose startPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        if (!debugMode) startPose = Settings.get("pose", new Pose(X_OFFSET, Y_OFFSET));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setPose(startPose);
        follower.setMaxPower(1);

//        IShooterCalculator shooterCalc = new LookupTableCalculator(ShooterCoefficients.VEL_COEFFS);
        IShooterCalculator shooterCalc = new LookupTableCalculator(ShooterCoefficients.CLOSE_VEL_COEFFS, ShooterCoefficients.FAR_VEL_COEFFS);

        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);
        kickstand = new Kickstand(hardwareMap);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::collect, intake, transfer))
                .whenReleased(new InstantCommand(intake::stop, intake, transfer));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::release, intake, transfer))
                .whenReleased(new InstantCommand(intake::stop, intake, transfer));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .and(new Trigger(this::isInsideLaunchZone))
                .whenActive(new ShootCommand(shooter, intake, transfer, drive, () -> follower.getPose().distanceFrom(alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL) / INCHES_TO_METERS >= DISTANCE_THRESHOLD_METERS));

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .and(new Trigger(this::isInsideLaunchZone))
                .whileActiveContinuous(new ShootCommand(shooter, intake, transfer, drive, () -> follower.getPose().distanceFrom(alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL) / INCHES_TO_METERS >= DISTANCE_THRESHOLD_METERS));

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

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(transfer::release),
                                new InstantCommand(intake::collect),
                                transfer.kick(),
                                new InstantCommand(intake::stop),
                                new InstantCommand(transfer::block)
                        )
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SHARE)
                .toggleWhenPressed(
                    new InstantCommand(() -> {
                        shooter.setHorizontalManualMode(true);
                        shooter.setVerticalManualMode(true);
                    }),
                    new InstantCommand(() -> {
                        shooter.setHorizontalManualMode(false);
                        shooter.setVerticalManualMode(false);
                    }
                )
        );

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 && (matchTime.isLessThan(20) || debugMode))
                .toggleWhenActive(
                        new InstantCommand(() -> {
                            kickstand.raise();
                            shooter.setHorizontalAngle(0);
                            shooter.disable();
                        }),
                        new InstantCommand(() -> {
                            kickstand.drop();
                            shooter.enable();
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(drive.goToGate());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(drive.goToBase());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                .whenPressed(drive.goToCenter());

        if (!tabletopMode && !debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(this::resetPoseToNearestCorner);
        }

        new Trigger(intake::isStalled)
                .whenActive(new InstantCommand(() -> gamepad1.rumble(300)));

        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.teleOpDrive(gamepad1),
                        drive
                )
        );
    }

    @Override
    public void run() {
        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());

        // Immediately cancel drive command if joysticks are moved
        boolean inputDetected = Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        if (!tabletopMode &&
                drive.getCurrentCommand() != null &&
                drive.getCurrentCommand() != drive.getDefaultCommand() &&
                inputDetected) {
            CommandScheduler.getInstance().cancel(drive.getCurrentCommand());
            follower.startTeleopDrive(USE_BRAKE_MODE);
        }

        // Cancel shooting if not in a launch zone
        if (!isInsideLaunchZone()) {
            CommandScheduler.getInstance().cancel(shooter.getCurrentCommand());
        }

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d robotPose = new Pose2d(-rotatedPose.getX() / INCHES_TO_METERS, -rotatedPose.getY() / INCHES_TO_METERS, new Rotation2d(rotatedPose.getHeading() - Math.PI));

        Pose rotatedGoalPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d goalPose = new Pose2d(-rotatedGoalPose.getX() / INCHES_TO_METERS, -rotatedGoalPose.getY() / INCHES_TO_METERS, new Rotation2d());

        telemetry.addData("!Reached RPM", shooter.reachedRPM());
        telemetry.addData("!Detected artifact", transfer.isArtifactDetected());
        telemetry.addData("!Inside LAUNCH ZONE", isInsideLaunchZone());
        telemetry.addData("!Reached angle", shooter.reachedAngle());
        telemetry.addData("!Can shoot", shooter.getCanShoot());

        telemetry.addData("Goal x", goalPose.getX());
        telemetry.addData("Goal y", goalPose.getY());
        telemetry.addData("Goal heading", 0);

        telemetry.addData("Pedro Robot x", follower.getPose().getX());
        telemetry.addData("Pedro Robot y", follower.getPose().getY());
        telemetry.addData("Pedro Robot heading", follower.getPose().getHeading());
        telemetry.addData("Robot x", robotPose.getX());
        telemetry.addData("Robot y", robotPose.getY());
        telemetry.addData("Robot heading", robotPose.getRotation().getDegrees());
        telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
        telemetry.addData("Distance from GOAL", follower.getPose().distanceFrom(alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL) / INCHES_TO_METERS);
        telemetry.addData("Current Voltage", voltageSensor.getVoltage());
        telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret target (deg)", Math.toDegrees(shooter.wrapped));
        telemetry.addData("Turret error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));

        telemetry.addData("hood pos", shooter.getRawHoodPosition());
        telemetry.addData("hood angle(deg)", shooter.getHoodAngleDegrees());
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.addData("Filtered Flywheel RPM", shooter.getFilteredRPM());
        telemetry.addData("Target RPM", shooter.solution.getRPM());
        telemetry.addData("Flywheel error: ", Math.abs(shooter.getRPM() - shooter.solution.getRPM()));
        telemetry.addData("Recovery Time", shooter.getRecoveryTime());
        telemetry.addData("calculating recovery", shooter.calculatedRecovery);

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
        Logger.recordOutput("Can Shoot", shooter.getCanShoot());
        Logger.recordOutput("Goal Pose", goalPose);
        Logger.recordOutput("Distance From Pose", follower.getPose().distanceFrom(alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL) / INCHES_TO_METERS);
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
        Logger.recordOutput("Shooter/Flywheel Filtered RPM", shooter.getFilteredRPM());
        Logger.recordOutput("Shooter/Flywheel Error", Math.abs(shooter.getRPM() - shooter.solution.getRPM()));
        Logger.recordOutput("Shooter/Flywheel Target", shooter.getTargetRPM());
        Logger.recordOutput("Shooter/Hood Raw Position", shooter.getRawHoodPosition());
        Logger.recordOutput("Shooter/Hood Angle (deg)", shooter.getHoodAngleDegrees());
        Logger.recordOutput("Turret/Turret Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        Logger.recordOutput("Turret/Turret Angle Target (deg)", Math.toDegrees(shooter.wrapped));
        Logger.recordOutput("Turret/Turret Angle Error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));
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
