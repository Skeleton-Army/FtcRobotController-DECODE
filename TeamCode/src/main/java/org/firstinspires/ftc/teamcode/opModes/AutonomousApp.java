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
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
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
import java.util.function.Consumer;
import java.util.function.Supplier;

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
    private Pose pushStartingPose;

    private final PathChain[] farPaths = new PathChain[4];
    private final PathChain[] nearPaths = new PathChain[4];
    private PathChain farDriveBackEnd;
    private PathChain nearDriveBackEnd;
    private PathChain spike3Open;
    private PathChain spike4Open;
    private PathChain pushPath;

    private Alliance alliance;
    private StartingPosition startingPosition;
    private List<Integer> pickupOrder;
    private int gateSpike;
    private boolean push;

    public PathChain farDriveBack() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                getRelative(new Pose(56.6, 15.862))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();
    }

    public PathChain nearDriveBack() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                getRelative(new Pose(42.920, 104.190))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();
    }

    public void setupPaths() {
        farStartingPose = getRelative(new Pose(56.6,8.7, Math.toRadians(180)));
        nearStartingPose = getRelative(new Pose(20.3, 123.6, Math.toRadians(142)));
        pushStartingPose = getRelative(new Pose(57.85,8.7, Math.toRadians(180)));

        Pose spike1End = getRelative(new Pose(12.330, 7.464));
        Pose spike2End = getRelative(new Pose(15.550755939524837, 35.76673866090713));
        Pose spike3End = getRelative(new Pose(15.086, 57.227));
        Pose spike4End = getRelative(new Pose(23.216, 83.663));
        Pose openGateEnd = getRelative(new Pose(14.572, 74));

        farPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                spike1End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        farPaths[1] = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    follower::getPose,
                                    getRelative(new Pose(70.600, 40.121)),
                                    spike2End
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
                                    spike3End
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
                                spike4End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        farDriveBackEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(45, 20))
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
                                spike1End
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
                                spike2End
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
                                spike3End
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
                                getRelative(new Pose(60.907, 79.931)),
                                spike4End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        nearDriveBackEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(53.716, 112.505))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        spike3Open = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(36.818, 57.39)),
                                openGateEnd
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        spike4Open = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(33.594, 76.111)),
                                openGateEnd
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        pushPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(30,8.7, Math.toRadians(180)))
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
        gateSpike = prompter.getOrDefault("gate_spike", -1);
        push = prompter.getOrDefault("push", false);
        Settings.set("alliance", alliance);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.addData("Pickup Order", pickupOrder);

        follower = Constants.createFollower(hardwareMap);

        shooter = new Shooter(hardwareMap, follower.poseTracker, new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS, ShooterCoefficients.VEL_COEFFS), alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower);

        setupPaths();

        Pose startingPose = startingPosition == StartingPosition.FAR ? farStartingPose : nearStartingPose;
        if (push) startingPose = pushStartingPose;

        follower.setStartingPose(startingPose);
    }

    @Override
    public void initialize() {
        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("starting_position", new OptionPrompt<>("SELECT STARTING POSITION", StartingPosition.FAR, StartingPosition.CLOSE))
                .prompt("pickup_order", new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 0, 1, 2, 3, 4))
                .prompt("open_gate", new BooleanPrompt("OPEN GATE?", false))
                .prompt("gate_spike",
                        () -> {
                            if (prompter.get("open_gate").equals(true)) {
                                return new OptionPrompt<>("AFTER WHICH SPIKE MARK?", 3, 4);
                            }
                            return null;
                        }
                )
                .prompt("push",
                        () -> {
                            if (prompter.get("starting_position").equals(StartingPosition.FAR)) {
                                new BooleanPrompt("PUSH OTHER ROBOT?", false);
                            }
                            return null;
                        }
                )
                .onComplete(this::afterPrompts);
    }

    @Override
    public void onStart() {
        Supplier<PathChain> driveBack =
                startingPosition == StartingPosition.FAR
                        ? this::farDriveBack
                        : this::nearDriveBack;

        Supplier<PathChain> finalDriveBack =
                startingPosition == StartingPosition.FAR
                        ? driveBack
                        : () -> nearDriveBackEnd;

        SequentialCommandGroup seq = new SequentialCommandGroup();

        if (push) {
            seq.addCommands(
                    new FollowPathCommand(follower, pushPath)
            );
        }

        seq.addCommands(
                new FollowPathCommand(
                        follower,
                        driveBack.get()
                ),
                new ShootCommand(shooter, intake, transfer, drive)
        );

        for (int i = 0; i < pickupOrder.size(); i++) {
            int spikeNumber = pickupOrder.get(i);
            boolean isLast = (i == pickupOrder.size() - 1);

            PathChain[] sourcePaths = startingPosition == StartingPosition.FAR
                    ? farPaths
                    : nearPaths;

            PathChain selectedPath = sourcePaths[spikeNumber - 1];

            seq.addCommands(
                    new InstantCommand(() -> intake.collect()),
                    new InstantCommand(() -> telemetry.addData("Current", spikeNumber)),
                    new FollowPathCommand(follower, selectedPath),
                    new InstantCommand(() -> intake.stop())
            );

            if (spikeNumber == gateSpike) {
                seq.addCommands(
                        new InstantCommand(() -> telemetry.addData("Current", "Opening gate")),
                        new FollowPathCommand(follower, spikeNumber == 3 ? spike3Open : spike4Open),
                        new WaitCommand(500)
                );
            }

            seq.addCommands(
                    new InstantCommand(() -> telemetry.addData("Current", "Driving back")),
                    new DeferredCommand(
                            () -> new FollowPathCommand(follower, isLast ? finalDriveBack.get() : driveBack.get()),
                            null
                    ),
                    new ShootCommand(shooter, intake, transfer, drive)
            );
        }

        if (startingPosition == StartingPosition.FAR) {
            seq.addCommands(new FollowPathCommand(follower, farDriveBackEnd));
        }

        seq.addCommands(new InstantCommand(this::requestOpModeStop));

        schedule(seq);
    }

    @Override
    public void run() {
        telemetry.update();
    }

    @Override
    public void initialize_loop() {
        prompter.run();
    }

    @Override
    public void end() {
        Settings.set("pose", follower.getPose(), false);
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
