package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.StartingPosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

import java.util.Collections;
import java.util.List;

@Autonomous
public class AutonomousApp extends ComplexOpMode {
    private final Prompter prompter = new Prompter(this);

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;

    private Pose farStartingPose;
    private Pose nearStartingPose;

    private final PathChain[] farPaths = new PathChain[4];
    private final PathChain[] nearPaths = new PathChain[4];
    private PathChain farDriveBack;
    private PathChain nearDriveBack;

    private Alliance alliance;
    private StartingPosition startingPosition;
    private List<Integer> pickupOrder;

    public void setupPaths() {
        farStartingPose = getRelative(new Pose(56.6,8.7, Math.toRadians(180)));
        nearStartingPose = getRelative(new Pose(17.8, 121, Math.toRadians(144)));

        farPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(8.708, 102.013)),
                                getRelative(new Pose(12.330, 7.464))
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        farPaths[1] = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    getRelative(new Pose(70.600, 40.121)),
                                    getRelative(new Pose(22.283, 35.456))
                            )
                    )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        farPaths[2] = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    getRelative(new Pose(80.864, 65.313)),
                                    getRelative(new Pose(12.086, 57.227))
                            )
                    )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();


        farPaths[3] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(80.864, 92.060)),
                                getRelative(new Pose(23.216, 83.663))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        farDriveBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(61.270, 15.862))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        nearPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(9.952, 45.408)),
                                getRelative(new Pose(9.952, 11.197))

                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        nearPaths[1] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(69.045, 32.968)),
                                getRelative(new Pose(21.149, 35.145))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        nearPaths[2] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(77.132, 57.538)),
                                getRelative(new Pose(21.149, 59.715))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        nearPaths[3] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(84.907, 79.931)),
                                getRelative(new Pose(21.149, 83.974))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        nearDriveBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(42.920, 104.190))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();
    }

    public void afterPrompts() {
        alliance = prompter.get("alliance");
        startingPosition = prompter.get("starting_position");
        pickupOrder = prompter.get("pickup_order");
        Settings.set("alliance", alliance);

        follower = Constants.createFollower(hardwareMap);

        shooter = new Shooter(hardwareMap, follower.poseTracker, new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS, ShooterCoefficients.VEL_COEFFS), alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower);

        setupPaths();

        follower.setStartingPose(startingPosition == StartingPosition.FAR ? farStartingPose : nearStartingPose);
    }

    @Override
    public void initialize() {
        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("starting_position", new OptionPrompt<>("SELECT STARTING POSITION", StartingPosition.FAR, StartingPosition.CLOSE))
                .prompt("pickup_order", new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 0, 1, 2, 3, 4))
                .onComplete(this::afterPrompts);
    }

    @Override
    public void onStart() {
        SequentialCommandGroup seq = new SequentialCommandGroup(
                new FollowPathCommand(
                        follower,
                        startingPosition == StartingPosition.FAR ? farDriveBack : nearDriveBack
                ),
                new ShootCommand(3, shooter, intake, transfer, drive)
        );

        for (Integer index : pickupOrder) {
            int i = index - 1;

            PathChain[] sourcePaths = startingPosition == StartingPosition.FAR
                    ? farPaths
                    : nearPaths;

            PathChain selectedPath = sourcePaths[i];

            seq.addCommands(
                    new InstantCommand(() -> intake.collect()),
                    new InstantCommand(() -> telemetry.addData("Current", i)),
                    new FollowPathCommand(follower, selectedPath),
                    new InstantCommand(() -> intake.stop()),
                    new InstantCommand(() -> telemetry.addData("Current", "Driving back")),
                    new DeferredCommand(
                            () -> {
                                PathChain parking = follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        follower.getPose(),
                                                        getRelative(new Pose(61.270, 15.862))
                                                )
                                        )
                                        .setConstantHeadingInterpolation(
                                                getRelative(Math.toRadians(180))
                                        )
                                        .build();

                                return new FollowPathCommand(follower, parking);
                            },
                            Collections.EMPTY_LIST
                    ),
                    new ShootCommand(3, shooter, intake, transfer, drive)
            );
        }

        schedule(seq);
    }

    @Override
    public void run() {
        follower.update();
        telemetry.addData("Current path: ", follower.getCurrentPathChain());
        telemetry.update();
    }

    @Override
    public void initialize_loop() {
        prompter.run();
    }

    private Pose getRelative(Pose originalPose) {
        if (alliance == Alliance.RED) {
            return originalPose.mirror();
        }

        return originalPose;
    }

    private double getRelative(double headingRad) {
        if (alliance == Alliance.RED) {
            return MathFunctions.normalizeAngle(Math.PI - headingRad);
        }

        return headingRad;
    }
}
