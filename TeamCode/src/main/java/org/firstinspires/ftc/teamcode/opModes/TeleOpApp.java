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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
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
        follower.setPose(new Pose(72, 72));

        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.hoodCoeffs, ShooterCoefficients.velCoeffs);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, Alliance.BLUE);
        intake = new Intake(hardwareMap);

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

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
//                .whenPressed(new FollowPathCommand(follower, parking));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new InstantCommand(() -> shooter.toggleTransfer(true)))
                .whenReleased(new InstantCommand(() -> shooter.toggleTransfer(false)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new InstantCommand(() -> shooter.kick()));
    }

    @Override
    public void run() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        if (gamepadEx1.isDown(GamepadKeys.Button.DPAD_UP)) {
            //shooter.setRawHoodPosition(MathFunctions.clamp(shooter.getRawHoodPosition() + 0.05, ShooterConfig.HOOD_POSSIBLE_MIN, 1));
            shooter.setVerticalAngle(50);
        }

        if (gamepadEx1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            //shooter.setRawHoodPosition(MathFunctions.clamp(shooter.getRawHoodPosition() - 0.05, ShooterConfig.HOOD_POSSIBLE_MIN, 1));
            shooter.setVerticalAngle(60);
        }

        if (gamepadEx1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            //shooter.setRawHoodPosition(MathFunctions.clamp(shooter.getRawHoodPosition() - 0.05, ShooterConfig.HOOD_POSSIBLE_MIN, 1));
            shooter.setVerticalAngle(30);
        }

        double inchesToMeters = 39.37;

        telemetry.addData("Robot x", follower.getPose().getX());
        telemetry.addData("Robot y", follower.getPose().getY());
        telemetry.addData("Robot heading", follower.getPose().getHeading());
        telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
        telemetry.addData("Turret position", shooter.getTurretPosition());
        telemetry.addData("Turret angle (rad)", shooter.getTurretAngle(AngleUnit.RADIANS));
        telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("hood pos", shooter.getRawHoodPosition());
        telemetry.addData("hood angle(deg)", (-34.7) * shooter.getRawHoodPosition() + 62.5);
        telemetry.addData("solution angle(rad)", shooter.solution.getVerticalAngle());
        telemetry.addData("solution angle(deg)", Math.toDegrees(shooter.solution.getVerticalAngle()));
        telemetry.addData("distance from goal: ", follower.getPose().distanceFrom(GoalPositions.BLUE_GOAL) / inchesToMeters);
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.addData("Recovery Time", shooter.getRecoveryTime());
        //telemetry.addData("Intake RPM", intake.getRPM());
        //telemetry.addData("PodX ticks", follower.getPoseTracker().getLocalizer().getLateralMultiplier());
        //telemetry.addData("PodY ticks", follower.getPoseTracker().getLocalizer().getForwardMultiplier());
        telemetry.update();

        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("Robot Pose", robotPose);
        //Logger.recordOutput("Shooter/Turret Target", Shooter.calculateTurretAngle(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
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
