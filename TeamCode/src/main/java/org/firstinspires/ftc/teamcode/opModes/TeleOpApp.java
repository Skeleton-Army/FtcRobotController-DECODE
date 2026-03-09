package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;
import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.DISTANCE_THRESHOLD_METERS;
import static org.firstinspires.ftc.teamcode.consts.ShooterConsts.SHOT_LATENCY;

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
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.settings.Settings;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
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
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.firstinspires.ftc.teamcode.utilities.FollowerManager;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

@TeleOp
public class TeleOpApp extends ComplexOpMode {
    public static final double INCHES_TO_METERS = 39.37;
    public static final double ROBOT_WIDTH = 16.53; // Side-to-side
    public static final double ROBOT_LENGTH = 14.96; // Front-to-back
    public static final double X_OFFSET = ROBOT_LENGTH / 2.0;
    public static final double Y_OFFSET = ROBOT_WIDTH / 2.0;

    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(ROBOT_LENGTH, ROBOT_WIDTH); // Length maps to X-axis and width maps to Y-axis relative to 0° heading
    private final PolygonZone futureRobotZone = new PolygonZone(ROBOT_LENGTH, ROBOT_WIDTH); // Length maps to X-axis and width maps to Y-axis relative to 0° heading

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;
    private Kickstand kickstand;
    private Vision vision;

    private VoltageSensor voltageSensor;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private boolean debugMode;
    private boolean tabletopMode;
    private Alliance alliance;

    private TimerEx matchTime;
    private TimerEx overrideTimer;

    private boolean isOverrideActive = false;

    private double lastLoopTime = 0;
    private int loopCount = 0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        matchTime = new TimerEx(120);
        overrideTimer = new TimerEx(1);

        debugMode = Settings.get("debug_mode", false);
        tabletopMode = Settings.get("tabletop_mode", false);
        alliance = Settings.get("alliance", Alliance.RED);

        follower = FollowerManager.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setMaxPower(1);

        Pose startPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        if (debugMode) follower.setPose(startPose);

//        IShooterCalculator shooterCalc = new LookupTableCalculator(ShooterCoefficients.VEL_COEFFS);
        //IShooterCalculator shooterCalc = new LookupTableCalculator(ShooterCoefficients.CLOSE_VEL_COEFFS, ShooterCoefficients.FAR_VEL_COEFFS);
        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS);

        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);
        kickstand = new Kickstand(hardwareMap);
        vision = new Vision(hardwareMap, follower.poseTracker);

        vision.addRelocalizationListener(drive::holdPoint); // Hold the new pose after relocalizing

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(intake::collect, intake));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(intake::release, intake, transfer));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new InstantCommand(() -> {
                    if (isShootingAllowed()) {
                        schedule(new ShootCommand(shooter, intake, transfer, drive, () -> follower.getPose().distanceFrom(alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL) / INCHES_TO_METERS >= DISTANCE_THRESHOLD_METERS));
                    } else {
                        gamepad1.rumble(300);
                        isOverrideActive = true;
                        overrideTimer.restart();
                    }
                }));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(transfer.kick());

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .and(new Trigger(this::isShootingAllowed))
                .and(new Trigger(transfer::isArtifactDetected))
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
                            drive.disable();
                        }),
                        new InstantCommand(() -> {
                            kickstand.drop();
                            shooter.enable();
                            drive.enable();
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(drive.goToGate());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(drive.goToBase());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                .whenPressed(drive.goToCenter());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> {
                    boolean success = vision.relocalize();
                    if (!success) gamepad1.rumble(300);
                });

        if (debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    .toggleWhenPressed(
                            new InstantCommand(() -> shooter.disable()),
                            new InstantCommand(() -> shooter.enable())
                    );
        }

        if (!tabletopMode && !debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(this::resetPoseToNearestCorner);
        }

        new Trigger(transfer.threeArtifactsDetected(intake::isCollecting, 250))
                .whenActive(new InstantCommand(() -> gamepad1.rumble(300)));

        intake.setDefaultCommand(new RunCommand(intake::stop, intake));
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.teleOpDrive(gamepad1),
                        drive
                )
        );
    }

    @Override
    public void onStart() {
        matchTime.start();
    }

    @Override
    public void run() {
        double currentTime = matchTime.getElapsed();
        double loopTimeMs = (currentTime - lastLoopTime) * 1000.0;
        lastLoopTime = currentTime;
        loopCount++;

        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());

        boolean isInsideLaunchZone = isInsideLaunchZonePredictive();
        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        shooter.setUpdateFlywheel(distanceFromLaunchZone() < 20);

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

        // Cancel shooting if:
        // 1. We aren't in the zone
        // 2. AND we haven't explicitly overriden for this specific shot cycle
        if (shooter.getCurrentCommand() == null && overrideTimer.isDone()) {
            isOverrideActive = false;
        }

        if (!isInsideLaunchZone && !isOverrideActive) {
            CommandScheduler.getInstance().cancel(shooter.getCurrentCommand());
        }

        double goalDistance = follower.getPose().distanceFrom(
                alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL
        ) / INCHES_TO_METERS;

        if (debugMode && loopCount % 5 == 0) {
            Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);

            telemetry.addData("!Loop Time (ms)", "%.2f", loopTimeMs);
            telemetry.addData("!Frequency (Hz)", "%.1f", 1000.0 / loopTimeMs);

            telemetry.addData("!Inside LAUNCH ZONE", isInsideLaunchZone);
            telemetry.addData("!Reached angle", shooter.reachedAngle());
            telemetry.addData("!can Shoot RPM calc ", shooter.getCanShootRPMCalc());
            telemetry.addData("!Can shoot", shooter.getCanShoot());

            telemetry.addData("Time remaining", matchTime.getRemaining());

            double driftX = Math.abs(X_OFFSET - follower.getPose().getX());
            double driftY = Math.abs(Y_OFFSET - follower.getPose().getY());
            telemetry.addData("Drift x", driftX);
            telemetry.addData("Drift y", driftY);
            telemetry.addData("Drift total", driftX + driftY);

            telemetry.addData("Pedro Robot x", follower.getPose().getX());
            telemetry.addData("Pedro Robot y", follower.getPose().getY());
            telemetry.addData("Pedro Robot heading", follower.getPose().getHeading());
            telemetry.addData("Robot x", -rotatedPose.getX());
            telemetry.addData("Robot y", -rotatedPose.getY());
            telemetry.addData("Robot heading", rotatedPose.getHeading() - Math.PI);
            telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
            telemetry.addData("Distance from GOAL", goalDistance);
            telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
            telemetry.addData("Turret target (deg)", Math.toDegrees(shooter.wrapped));
            telemetry.addData("Turret target solution (deg)", Math.toDegrees(shooter.solution.getHorizontalAngle()));
            telemetry.addData("Turret error (deg)", Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES));
            telemetry.addData("Turret window (deg)", Math.toDegrees(shooter.getTurretWindow()));

            telemetry.addData("hood pos", shooter.getRawHoodPosition());
            telemetry.addData("hood angle(deg)", shooter.getHoodAngleDegrees());
            telemetry.addData("Flywheel RPM", shooter.getRPM());
            telemetry.addData("Filtered Flywheel RPM", shooter.filteredRPM);
            telemetry.addData("Target solution RPM", shooter.solution.getRPM());
            telemetry.addData("Flywheel error: ", Math.abs(shooter.getRPM() - shooter.getTargetRPM()));

            telemetry.addData("Shot Hood Angle", shooter.shotHoodAngle);
            telemetry.addData("Shot Turret Angle", shooter.shotTurretAngle);
            telemetry.addData("Shot Flywheel RPM", shooter.shotFlywheelRPM);
            telemetry.addData("Shot goal distance", shooter.shotGoalDistance);
            telemetry.addData("robot vel x ", follower.getVelocity().getXComponent());
            telemetry.addData("robot vel y ", follower.getVelocity().getYComponent());

            telemetry.update();
        }

        if (loopCount % 2 == 0) {
            Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
            Pose2d robotPose = new Pose2d(-rotatedPose.getX() / INCHES_TO_METERS, -rotatedPose.getY() / INCHES_TO_METERS, new Rotation2d(-rotatedPose.getHeading()));

            Logger.recordOutput("Diagnostics/LoopTimeMs", loopTimeMs);
            Logger.recordOutput("Diagnostics/Hz", 1000.0 / loopTimeMs);
            Logger.recordOutput("Robot Pose", robotPose);
            Logger.recordOutput("Voltage", voltage);
            Logger.recordOutput("Inside LAUNCH ZONE", isInsideLaunchZone);
            Logger.recordOutput("Reached Angle", shooter.reachedAngle());
            Logger.recordOutput("Can Shoot RPM calc", shooter.getCanShootRPMCalc());
            Logger.recordOutput("Can Shoot", shooter.getCanShoot());
            Logger.recordOutput("Distance From Pose", goalDistance);
            Logger.recordOutput("Shooter/Flywheel/ Filtered RPM", shooter.filteredRPM);
            Logger.recordOutput("Shooter/Flywheel/Error", Math.abs(shooter.filteredRPM - shooter.getTargetRPM()));
            Logger.recordOutput("Shooter/Flywheel/ Target", shooter.getTargetRPM());
            Logger.recordOutput("Shooter/Hood/ Raw Position", shooter.getRawHoodPosition());
            Logger.recordOutput("Shooter/Hood/ Angle (deg)", shooter.getHoodAngleDegrees());
            Logger.recordOutput("Turret/Turret/ Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
            Logger.recordOutput("Turret/Turret/ Angle Target (deg)", Math.toDegrees(shooter.wrapped));
            Logger.recordOutput("Turret/Turret/ Angle Error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));
            Logger.recordOutput("Turret/Turret/ Turret window (deg)", Math.toDegrees(shooter.getTurretWindow()));
        }
    }

    private boolean isShootingAllowed() {
        return isInsideLaunchZonePredictive() || isOverrideActive;
    }

    public boolean isInsideLaunchZone() {
        boolean insideClose = robotZone.isInside(closeLaunchZone);
        boolean insideFar = robotZone.isInside(farLaunchZone);
        return insideClose || insideFar;
    }

    public boolean isInsideLaunchZonePredictive() {
        // Project future X and Y based on current velocity
        double futureX = follower.getPose().getX() + (follower.getVelocity().getXComponent() * SHOT_LATENCY);
        double futureY = follower.getPose().getY() + (follower.getVelocity().getYComponent() * SHOT_LATENCY);

        // Create a temporary zone for the future position
        futureRobotZone.setPosition(futureX, futureY);
        futureRobotZone.setRotation(follower.getPose().getHeading());

        return futureRobotZone.isInside(closeLaunchZone) || futureRobotZone.isInside(farLaunchZone);
    }

    public double distanceFromLaunchZone() {
        return Math.min(robotZone.distanceTo(closeLaunchZone), robotZone.distanceTo(farLaunchZone));
    }

    private void resetPoseToNearestCorner() {
        Pose currentPose = follower.getPose();

        Pose newPose;
        if (alliance == Alliance.RED) {
            newPose = new Pose(X_OFFSET, Y_OFFSET);
        } else {
            newPose = new Pose(141.5 - X_OFFSET, Y_OFFSET);
        }

        follower.setPose(new Pose(newPose.getX(), newPose.getY(), currentPose.getHeading()));
        follower.startTeleopDrive(USE_BRAKE_MODE);
    }
}
