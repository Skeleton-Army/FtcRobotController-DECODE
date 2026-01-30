package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.consts.ShooterCoefficients.DISTANCE_THRESHOLD_METERS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.LookupTableCalculator;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

@TeleOp
public class ShooterTest extends ComplexOpMode   {
    private final double INCHES_TO_METERS = 39.37;

    private final Pose[] points = new Pose[16];
    private int index = 0;

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;

    private PathChain YN(Pose point)
    {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                point
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
    }

    private void forNextPath()
    {
        PathChain path = YN(points[index]);
        follower.followPath(path);
        index++;

        if (index >= points.length)
        {
            index = 0;
        }
    }

    public void setupPaths() {
        // Far points <3
        points[0] = new Pose(65.313, 10.263);
        points[1] = new Pose(67.490, 15.551);
        points[2] = new Pose(85.529, 15.551);
        points[3] = new Pose(72.467, 22.704);
        points[4] = new Pose(72.156, 9.330);
        points[5] = new Pose(72.156, 0.622);

        // Near points <3
        points[6] = new Pose(35.456, 132.803);
        points[7] = new Pose(60.026, 130.937);
        points[8] = new Pose(84.285, 131.559);
        points[9] = new Pose(107.922, 131.559);
        points[10] = new Pose(107.611, 106.989);
        points[11] = new Pose(83.974, 107.611);
        points[12] = new Pose(59.404, 106.989);
        points[13] = new Pose(35.456, 107.611);
        points[14] = new Pose(59.715, 83.663);
        points[15] = new Pose(84.596, 83.663);

        // googoogaga
    }

    @Override
    public void initialize() {
        Alliance alliance = Alliance.RED;
        IShooterCalculator shooterCalc = new LookupTableCalculator(ShooterCoefficients.CLOSE_VEL_COEFFS, ShooterCoefficients.FAR_VEL_COEFFS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7.87, 8.26, Math.toRadians(0)));

        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenActive(new ShootCommand(shooter, intake, transfer, drive, () -> follower.getPose().distanceFrom(alliance == Alliance.RED ? GoalPositions.RED_GOAL_FAR : GoalPositions.BLUE_GOAL_FAR) / INCHES_TO_METERS >= DISTANCE_THRESHOLD_METERS));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenActive(this::forNextPath);

        setupPaths();
    }

    @Override
    public void run() {
        follower.update();

        telemetry.addData("Target Point", index);
        telemetry.addData("Target X", points[index].getX());
        telemetry.addData("Target Y", points[index].getY());
        telemetry.update();
    }
}
