package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.consts.CloseShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.FarShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.LaunchZone;
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

    private double totalTraveledX = 0;
    private double totalTraveledY = 0;

    private boolean autoFireEnabled = true;

    private final TimerEx zoneExitTimer = new TimerEx(0.3);
    private boolean zoneExitTimerRunning = false;

    private boolean gemsGoal = false;

    Pose startPose;

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

//        startPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        startPose = new Pose(GoalPositions.FIELD_LENGTH - X_OFFSET, Y_OFFSET, Math.toRadians(180)); // starts it on the bottom-right corner
        if (debugMode) follower.setPose(startPose);

        IShooterCalculator shooterCalcClose = new ShooterCalculator(new CloseShooterCoefficients());
        IShooterCalculator shooterCalcFar = new ShooterCalculator(new FarShooterCoefficients());
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalcClose, shooterCalcFar, alliance);

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);
        kickstand = new Kickstand(hardwareMap);
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.APRILTAG_PIPELINE);

        vision.addRelocalizationListener(drive::holdPoint); // Hold the new pose after relocalizing

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        autoFireEnabled = Settings.get("auto_fire", true);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .and(new Trigger(() -> intake.getCurrentCommand() == null || intake.getCurrentCommand() == intake.getDefaultCommand()))
                .whileActiveContinuous(new InstantCommand(intake::collect, intake));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(new Trigger(() -> intake.getCurrentCommand() == null || intake.getCurrentCommand() == intake.getDefaultCommand()))
                .whileActiveContinuous(new InstantCommand(intake::release, intake, transfer));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new InstantCommand(() -> {
                    if (isShootingAllowed()) {
                        schedule(new ShootCommand(shooter, intake, transfer, drive, gemsGoal ? IntakeConfig.SLOW_SHOOTING_POWER : IntakeConfig.SHOOTING_POWER, 2000));
                    } else {
                        gamepad1.rumble(300);
                        isOverrideActive = true;
                        overrideTimer.restart();
                    }
                }));

        // Fire immediately when entering zone
        new Trigger(() -> autoFireEnabled
                && !gemsGoal
                && drive.isInsideLaunchZonePredictive()
                && shooter.getCanShoot()
                && !isShootingBlocked()
                && (shooter.getCurrentCommand() == null || shooter.getCurrentCommand() == shooter.getDefaultCommand())
                && (transfer.getCurrentCommand() == null || transfer.getCurrentCommand() == transfer.getDefaultCommand())
                && (intake.getCurrentCommand() == null || intake.getCurrentCommand() == intake.getDefaultCommand())
        )
                .whenActive(new ShootCommand(shooter, intake, transfer, drive));

        // Kick automatically when exiting the launch zone
//        new Trigger(drive::isInsideLaunchZonePredictive)
//                .whenInactive(transfer.kick());

        // Toggle auto-fire on-off
        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(
                        new InstantCommand(() -> {
                            autoFireEnabled = !autoFireEnabled;
                            gamepad1.rumble(200);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setVerticalOffset(shooter.getVerticalOffset() + 0.05))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setVerticalOffset(shooter.getVerticalOffset() - 0.05))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setHorizontalOffset(shooter.getHorizontalOffset() + 0.003))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setHorizontalOffset(shooter.getHorizontalOffset() - 0.003))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
                .toggleWhenPressed(
                        new InstantCommand(() -> {
                            shooter.setGoalPose(GoalPositions.BLUE_GOAL_GEMS,GoalPositions.RED_GOAL_GEMS);
                            shooter.setGoalPoseFar(GoalPositions.BLUE_GOAL_GEMS,GoalPositions.RED_GOAL_GEMS);
                            gemsGoal = true;

                        }),
                        new InstantCommand(() -> {
                            shooter.setGoalPose(GoalPositions.BLUE_GOAL,GoalPositions.RED_GOAL);
                            shooter.setGoalPoseFar(GoalPositions.BLUE_GOAL_FAR,GoalPositions.RED_GOAL_FAR);
                            gemsGoal = false;
                        })
                );

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .toggleWhenActive(
                        new InstantCommand(() -> {
                            kickstand.raise();
                            shooter.setHorizontalManualMode(true);
                            shooter.setHorizontalAngle(0);
                            shooter.disable();
                            drive.disable();
                        }),
                        new InstantCommand(() -> {
                            kickstand.drop();
                            shooter.setHorizontalManualMode(false);
                            shooter.enable();
                            drive.enable();
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(drive.goToBase());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SHARE)
                .whenPressed(drive.LoadingZoneCycle());


//        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(() -> {
//                    boolean success = vision.relocalize();
//                    if (!success) gamepad1.rumble(300);
//                });

        if (debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    .toggleWhenPressed(
                            new InstantCommand(() -> shooter.disable()),
                            new InstantCommand(() -> shooter.enable())
                    );
        }

        if (!tabletopMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(this::resetPoseToNearestCorner);
        }

        new Trigger(transfer.threeArtifactsDetected(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                200
        ))
                .whenActive(new InstantCommand(() -> gamepad1.rumble(300)));

        // Cancel shooting when turret wraps around
        new Trigger(shooter::getJustWrapped)
                .whenActive(new InstantCommand(() -> CommandScheduler.getInstance().cancel(shooter.getCurrentCommand())));

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

        if (loopCount > 1) {
            double deltaX = Math.abs(follower.poseTracker.getRawPose().getX() - follower.poseTracker.getPreviousPose().getX());
            double deltaY = Math.abs(follower.poseTracker.getRawPose().getY() - follower.poseTracker.getPreviousPose().getY());

            if (deltaX > 0.05) totalTraveledX += deltaX;
            if (deltaY > 0.05) totalTraveledY += deltaY;
        }

        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        shooter.setUpdateFlywheel(drive.distanceFromLaunchZone() < 40);
        shooter.setZoneCalculator(drive.distanceFromCloseLaunchZone() < drive.distanceFromFarLaunchZone() ? LaunchZone.CLOSE : LaunchZone.FAR);

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

        boolean insideZoneNow = drive.isInsideLaunchZonePredictive();

        if (insideZoneNow) {
            zoneExitTimerRunning = false;
        } else if (!zoneExitTimerRunning) {
            zoneExitTimer.restart();
            zoneExitTimerRunning = true;
        }

        boolean confirmedOutsideZone = zoneExitTimerRunning && zoneExitTimer.isDone();

        if (confirmedOutsideZone && !isOverrideActive) {
            Command currentShooterCommand = shooter.getCurrentCommand();

            if (currentShooterCommand instanceof ShootCommand) {
                CommandScheduler.getInstance().cancel(currentShooterCommand);

                new ParallelCommandGroup(
                        new InstantCommand(transfer::block, transfer),
                        transfer.kick(),
                        new SequentialCommandGroup(
                                new InstantCommand(intake::collect, intake),
                                new WaitCommand(50),
                                new InstantCommand(intake::stop, intake)
                        )

                ).schedule(false);
            }
        }

        double goalDistance = follower.getPose().distanceFrom(
                alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.RED_GOAL
        ) / INCHES_TO_METERS;

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        telemetry.addData("!Loop Time (ms)", "%.2f", loopTimeMs);
        telemetry.addData("!Frequency (Hz)", "%.1f", 1000.0 / loopTimeMs);

        telemetry.addData("!Inside LAUNCH ZONE", drive.isInsideLaunchZonePredictive());
        telemetry.addData("!Reached angle", shooter.reachedAngle());
        telemetry.addData("!can Shoot RPM calc ", shooter.getCanShootRPMCalc());
        telemetry.addData("!Can shoot", shooter.getCanShoot());
        telemetry.addData("!Is currently shooting", shooter.getCurrentCommand() != null);

        telemetry.addData("Time remaining", matchTime.getRemaining());

        double driftX = Math.abs(startPose.getX() - follower.getPose().getX());
        double driftY = Math.abs(startPose.getY() - follower.getPose().getY());
        telemetry.addData("Drift x", driftX);
        telemetry.addData("Drift y", driftY);
        telemetry.addData("Drift total", driftX + driftY);

        telemetry.addData("Total Traveled X", totalTraveledX);
        telemetry.addData("Total Traveled Y", totalTraveledY);
        telemetry.addData("Total Traveled XY", totalTraveledX + totalTraveledY);

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
        telemetry.addData("Turret horizontal offset (deg)", Math.toDegrees(shooter.getHorizontalOffset()));

        telemetry.addData("hood pos", shooter.getRawHoodPosition());
        telemetry.addData("hood angle(deg)", shooter.getHoodAngleDegrees());
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.addData("Filtered Flywheel RPM", shooter.filteredRPM);
        telemetry.addData("Target solution RPM", shooter.getTargetRPM());
        telemetry.addData("Flywheel error", Math.abs(shooter.getRPM() - shooter.getTargetRPM()));
        telemetry.addData("Intake RPM", intake.getRPM());

        telemetry.addData("Shot Hood Angle", shooter.shotHoodAngle);
        telemetry.addData("Shot Turret Angle", shooter.shotTurretAngle);
        telemetry.addData("Shot Flywheel RPM", shooter.shotFlywheelRPM);
        telemetry.addData("Shot goal distance", shooter.shotGoalDistance);
        telemetry.addData("robot vel x ", follower.getVelocity().getXComponent());
        telemetry.addData("robot vel y ", follower.getVelocity().getYComponent());

        telemetry.update();

        if (loopCount % 2 == 0) {
            Pose2d robotPose = new Pose2d(-rotatedPose.getX() / INCHES_TO_METERS, -rotatedPose.getY() / INCHES_TO_METERS, new Rotation2d(-rotatedPose.getHeading()));

            Logger.recordOutput("Diagnostics/LoopTimeMs", loopTimeMs);
            Logger.recordOutput("Diagnostics/Hz", 1000.0 / loopTimeMs);
            Logger.recordOutput("Robot Pose", robotPose);
            Logger.recordOutput("Voltage", voltage);
            Logger.recordOutput("Inside LAUNCH ZONE", drive.isInsideLaunchZonePredictive());
            Logger.recordOutput("Reached Angle", shooter.reachedAngle());
            Logger.recordOutput("Can Shoot RPM calc", shooter.getCanShootRPMCalc());
            Logger.recordOutput("Can Shoot", shooter.getCanShoot());
            Logger.recordOutput("Is Currently Shooting", shooter.getCurrentCommand() != null);
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
            Logger.recordOutput("Turret/Turret/ Turret offset (deg)", Math.toDegrees(shooter.getHorizontalOffset()));
        }
    }

    private boolean isShootingAllowed() {
        return !isShootingBlocked() && drive.isInsideLaunchZonePredictive() || isOverrideActive;
    }

    private void resetPoseToNearestCorner() {
        Pose newPose;
        if (alliance == Alliance.RED) {
            newPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        } else {
            newPose = new Pose(GoalPositions.FIELD_LENGTH - X_OFFSET, Y_OFFSET, Math.toRadians(180));
        }

        follower.setPose(new Pose(newPose.getX(), newPose.getY(), newPose.getHeading()));
        follower.startTeleopDrive(USE_BRAKE_MODE);
        gamepad1.rumble(300);
    }

    public boolean getShooterGoal(){
        return gemsGoal;
    }

    private boolean isShootingBlocked() {
        if (alliance == Alliance.BLUE) {
            if (gemsGoal)
                return !(follower.getPose().getX() <= getRelative(75) && follower.getPose().getY() >= 123);
            else
                return (follower.getPose().getX() >= getRelative(105) && follower.getPose().getY() > 142);
        } else {
            if (gemsGoal)
                return !(follower.getPose().getX() >= getRelative(75) && follower.getPose().getY() >= 123);
            else
                return (follower.getPose().getX() <= getRelative(105) && follower.getPose().getY() > 142);
        }
    }

    private double getRelative(double x){
        if(alliance == Alliance.RED)
            return GoalPositions.FIELD_LENGTH - x;
        return x;
    }

}
