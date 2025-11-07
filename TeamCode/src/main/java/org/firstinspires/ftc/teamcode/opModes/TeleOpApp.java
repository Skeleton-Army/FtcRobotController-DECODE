package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

@TeleOp
public class TeleOpApp extends ComplexOpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone blueBase = new PolygonZone(new Point(105.5, 33.5), 20, 20);
    private final PolygonZone redBase = new PolygonZone(new Point(38.5, 33.5), 20, 20);
    private final PolygonZone robotZone = new PolygonZone(18, 18);

    private Follower follower;
    private Intake intake;
    private Shooter shooter;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        follower = Constants.createFollower(hardwareMap);
//        follower.startTeleopDrive(true);
//        follower.setStartingPose(new Pose(72, 72));

//        intake = new Intake(hardwareMap);
        //shooter = new Shooter(hardwareMap, follower.poseTracker);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

//        new Trigger(() -> gamepad1.right_trigger > 0.1)
//                .whenActive(new ShootCommand(shooter));

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(new InstantCommand(() -> intake.set(-1)))
//                .whenReleased(new InstantCommand(() -> intake.set(0)));
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> intake.set(1)))
//                .whenReleased(new InstantCommand(() -> intake.set(0)));

        schedule(
                // TODO: Set shooter angle to GOAL
        );
    }

    @Override
    public void run() {
//        follower.update();
//        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

//        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
//        robotZone.setRotation(follower.getPose().getHeading());

        telemetry.addData("Is inside close LAUNCH ZONE?", robotZone.isInside(closeLaunchZone));
        telemetry.addData("Is fully inside close LAUNCH ZONE?", robotZone.isFullyInside(closeLaunchZone));
        telemetry.addData("Distance to close LAUNCH ZONE", robotZone.distanceTo(closeLaunchZone));
        telemetry.addLine();
        telemetry.addData("Is inside far LAUNCH ZONE?", robotZone.isInside(farLaunchZone));
        telemetry.addData("Is fully inside far LAUNCH ZONE?", robotZone.isFullyInside(farLaunchZone));
        telemetry.addData("Distance to far LAUNCH ZONE", robotZone.distanceTo(farLaunchZone));
        telemetry.addLine();
        telemetry.addData("Partially parked BLUE BASE?", robotZone.isInside(blueBase));
        telemetry.addData("Fully parked BLUE BASE?", robotZone.isFullyInside(blueBase));
        telemetry.addData("Distance to BLUE BASE", robotZone.distanceTo(blueBase));
        telemetry.addLine();
        telemetry.addData("Partially parked RED BASE?", robotZone.isInside(redBase));
        telemetry.addData("Fully parked RED BASE?", robotZone.isFullyInside(redBase));
        telemetry.addData("Distance to RED BASE", robotZone.distanceTo(redBase));
        telemetry.addLine();
//        telemetry.addData("Robot x", follower.getPose().getX());
//        telemetry.addData("Robot y", follower.getPose().getY());
//        telemetry.addData("Robot heading", follower.getPose().getHeading());
        telemetry.addData("Debug Mode", TestSettings.get("debug"));
//        Buh buh = TestSettings.get("buh");
//        telemetry.addData("Buh", buh);
        telemetry.update();

//        double inchesToMeters = 39.37;
//        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
//        Logger.recordOutput("Robot Pose", robotPose);
    }
}
