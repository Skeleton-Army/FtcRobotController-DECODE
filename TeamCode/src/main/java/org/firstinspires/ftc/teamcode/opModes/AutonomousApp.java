package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

import java.util.List;

@Autonomous
public class AutonomousApp extends ComplexOpMode {
    private final Prompter prompter = new Prompter(this);

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;

    private final PathChain[] farPaths = new PathChain[4];
    private final PathChain[] nearPaths = new PathChain[4];
    private PathChain farDriveBack;
    private PathChain nearDriveBack;

    private Alliance alliance;
    private StartingPosition startingPosition;
    private List<Integer> pickupOrder;

    public void setupPaths() {
        farPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(65.935, 33.901),
                                new Pose(63.447, 62.825),
                                new Pose(2.177, 74.022),
                                new Pose(10.575, 40.432),
                                new Pose(9.330, 9.952)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        farPaths[1] = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    new Pose(63.758, 36.700),
                                    new Pose(19.594, 36.078)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

        farPaths[2] = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    new Pose(69.045, 65.313),
                                    new Pose(15.551, 59.404)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();


        farPaths[3] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(78.376, 88.639),
                                new Pose(18.972, 83.974)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        farDriveBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                new Pose(61.270, 15.862)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(90)
                )
                .build();

        nearPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(9.019, 47.896),
                                new Pose(8.086, 8.708)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        nearPaths[1] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(45.097, 22.082),
                                new Pose(16.795, 37.011)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        nearPaths[2] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(43.231, 52.873),
                                new Pose(12.130, 59.093)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        nearPaths[3] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(43.231, 82.419),
                                new Pose(17.106, 83.352)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        nearDriveBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                new Pose(42.920, 104.190)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(90)
                )
                .build();
    }

    public void setupPrompts() {
        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("startPos", new OptionPrompt<>("SELECT STARTING POSITIONS", StartingPosition.FAR, StartingPosition.CLOSE))
                .prompt("pickup_order", new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 1, 2, 3, 4))
                .onComplete(() -> {
                    alliance = prompter.get("alliance");
                    startingPosition = prompter.get("startPos");
                    pickupOrder = prompter.get("pickup_order");

                    Settings.set("alliance", alliance);

                    shooter = new Shooter(hardwareMap, follower.poseTracker, new ShooterCalculator(ShooterCoefficients.hoodCoeffs, ShooterCoefficients.velCoeffs), alliance);
                });
    }

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.604751619870406,8.708423326133913, Math.toRadians(90)));

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        setupPaths();
        setupPrompts();
    }

    @Override
    public void onStart() {
        SequentialCommandGroup seq = new SequentialCommandGroup(
                new FollowPathCommand(
                        follower,
                        startingPosition == StartingPosition.FAR ? farDriveBack : nearDriveBack
                ),
                new ShootCommand(3, shooter, intake, transfer)
        );

        for (Integer index : pickupOrder) {
            int i = index - 1;

            PathChain[] sourcePaths = startingPosition == StartingPosition.FAR
                    ? farPaths
                    : nearPaths;

            PathChain selectedPath = sourcePaths[i];

            seq.addCommands(new InstantCommand(() -> intake.collect()));
            seq.addCommands(new FollowPathCommand(follower, selectedPath));
            seq.addCommands(new InstantCommand(() -> intake.stop()));
            seq.addCommands(new FollowPathCommand(
                    follower,
                    startingPosition == StartingPosition.FAR ? farDriveBack : nearDriveBack
            ));
            seq.addCommands(new ShootCommand(3, shooter, intake, transfer));
        }

        schedule(seq);
    }

    @Override
    public void run() {
        follower.update();
    }

    @Override
    public void initialize_loop() {
        prompter.run();
    }
}
