package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.StartingPosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class AutonomousApp extends ComplexOpMode {
    private final int[] ARTIFACTS_COUNT = {3, 6, 9, 12, 15, 30000};
    private final Prompter prompter = new Prompter(this);

    private Follower follower;
    private Intake intake;
    private PathChain batata;
    private PathChain potato;
    private PathChain avocado;
    private PathChain tomato;
    private Shooter shooter;

    private Alliance alliance;
    private StartingPosition startingPosition;
    private int artifacts;

    public void setupPaths() {
        batata = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.294, 10.263),
                                new Pose(57.538, 36.078),
                                new Pose(17.417, 36.078)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        potato = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17.417, 36.078),
                                new Pose(60.959, 29.235),
                                new Pose(56.294, 14.307)

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        avocado = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.294, 14.307),
                                new Pose(64.069, 57.227),
                                new Pose(8.708, 53.495)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        tomato = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.708, 53.495),
                                new Pose(53.184, 55.983),
                                new Pose(53.495, 9.952)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void setupPrompts() {
        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("startPos", new OptionPrompt<>("SELECT STARTING POSITIONS", StartingPosition.FAR, StartingPosition.CLOSE))
                .prompt("artifacts", new OptionPrompt<>("SELECT AMOUNT OF ARTIFACTS", ARTIFACTS_COUNT))
                .onComplete(() -> {
                    alliance = prompter.get("alliance");
                    startingPosition = prompter.get("startPos");
                    artifacts = prompter.get("artifacts");
                });
    }

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.604751619870406,7.464362850971918, Math.toRadians(90)));

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, follower.poseTracker);

        setupPaths();
        setupPrompts();
    }

    @Override
    public void onStart() {
        schedule(
                new SequentialCommandGroup(
                        new ShootCommand(shooter),
                        new InstantCommand(() -> intake.collect()),
                        new FollowPathCommand(follower, batata),
                        new InstantCommand(() -> intake.stop()),
                        new FollowPathCommand(follower, potato),
                        new ShootCommand(shooter),
                        new InstantCommand(() -> intake.collect()),
                        new FollowPathCommand(follower, avocado),
                        new InstantCommand(() -> intake.stop()),
                        new FollowPathCommand(follower, tomato)
                )
        );
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
