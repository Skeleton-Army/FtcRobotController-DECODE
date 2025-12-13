package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.OpModeManager;
import com.skeletonarmy.marrow.settings.Settings;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
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
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

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

    private boolean insideZone;

    @Override
    public void initialize() {
        debugMode = Settings.get("debug_mode", false);
        tabletopMode = Settings.get("tabletop_mode", false);
        alliance = Settings.get("alliance", Alliance.RED);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setPose(new Pose(72, 72));

        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS, ShooterCoefficients.VEL_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.collect(), intake))
                .whenReleased(new InstantCommand(() -> intake.stop(), intake));

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
                .whenPressed(new ShootCommand(3, shooter, intake, transfer, drive));

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whileActiveContinuous(new ShootCommand(1, shooter, intake, transfer, drive));

        if (!tabletopMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                    .whenPressed(drive.goToBase());
        }

        if (!tabletopMode && debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(drive.goToCenter());
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
        boolean insideClose = robotZone.isInside(closeLaunchZone);
        boolean insideFar = robotZone.isInside(farLaunchZone);
        boolean insideZone = insideClose || insideFar;

        if (!insideZone && !debugMode) {
            CommandScheduler.getInstance().cancel(shooter.getCurrentCommand());
        }

        double inchesToMeters = 39.37;

        telemetry.addData("Robot x", follower.getPose().getX());
        telemetry.addData("Robot y", follower.getPose().getY());
        telemetry.addData("Robot heading", follower.getPose().getHeading());
        telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
        telemetry.addData("!Inside LAUNCH ZONE", insideZone);
        telemetry.addData("Current Voltage", voltageSensor.getVoltage());
        //telemetry.addData("Sensor distance", transfer.getDistance());
        //telemetry.addData("Turret position", shooter.getTurretPosition());
        //telemetry.addData("Turret angle (rad)", shooter.getTurretAngle(AngleUnit.RADIANS));
        telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret error (deg)", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret solution error (deg)", shooter.solution.getHorizontalAngle() - shooter.getTurretAngle(AngleUnit.DEGREES));

        telemetry.addData("hood pos", shooter.getRawHoodPosition());
        telemetry.addData("hood angle(deg)", (-34.7) * shooter.getRawHoodPosition() + 62.5);
        //telemetry.addData("solution angle(rad)", shooter.solution.getVerticalAngle());
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
        //telemetry.addData("Intake RPM", intake.getRPM());
        //telemetry.addData("PodX ticks", follower.getPoseTracker().getLocalizer().getLateralMultiplier());
        //telemetry.addData("PodY ticks", follower.getPoseTracker().getLocalizer().getForwardMultiplier());
        telemetry.update();

        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("Robot Pose", robotPose);
        //Logger.recordOutput("Shooter/Turret Target", Shooter.calculateTurretAngle(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
    }
}
