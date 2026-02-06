package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
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
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.util.List;
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
    private final Supplier<PathChain>[] nearPathsReturn = new Supplier[4];
    private final Supplier<PathChain>[] farPathsReturn = new Supplier[4];
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

    private VoltageSensor voltageSensor;

    private Pose farDriveBack;
    private Pose nearDriveBack;

    private Pose gateOpenPose;

    public PathChain farDriveBack() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                farDriveBack
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
                                nearDriveBack
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();
    }

    public PathChain collectFromGate() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                getRelative(new Pose(46.462, 64.985)),
                                gateOpenPose
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        getRelative(Math.toRadians(140))
                )
                .build();
    }

    public PathChain backFromGateCollection() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                getRelative(new Pose(46.462, 64.985)),
                                farDriveBack
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        getRelative(Math.toRadians(180))
                )
                .build();
    }


    public void setupPaths() {
        farStartingPose = getRelative(new Pose(56.6,8.5, Math.toRadians(90)));
        nearStartingPose = getRelative(new Pose(19.623, 120.368, Math.toRadians(143)));
        pushStartingPose = getRelative(new Pose(57.85,8.5, Math.toRadians(180)));

        Pose spike1End = getRelative(new Pose(9.330697624190067, 9.708060475161995));
        Pose spike2End = getRelative(new Pose(15.550755939524837, 35.76673866090713));
        Pose spike3End = getRelative(new Pose(17.263, 58.026));
        Pose spike4End = getRelative(new Pose(23.216, 83.663));
        Pose openGateEnd = getRelative(new Pose(14.572, 74));

        farDriveBack = getRelative(new Pose(56.6, 15.862));
        nearDriveBack = getRelative(new Pose(56.605, 91.127));
        gateOpenPose = getRelative(new Pose(13.702, 62.125));

        nearPathsReturn[0] = this::nearDriveBack;
        nearPathsReturn[1] = this::nearDriveBack;
        nearPathsReturn[2] = () -> follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                getRelative(new Pose(30.655, 54.544)),
                                getRelative(new Pose(49.264, 66.136)),
                                nearDriveBack
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();
        nearPathsReturn[3] = this::nearDriveBack;

        farPathsReturn[0] = this::farDriveBack;
        farPathsReturn[1] = this::farDriveBack;
        farPathsReturn[2] = this::farDriveBack;
        farPathsReturn[3] = this::farDriveBack;

        farPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                spike1End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(13.330697624190067, 9.708060475161995))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .addPath(
                        new BezierLine(
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
                                getRelative(new Pose(76.510, 65.313)),
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
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(21.263, 58.026))
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

        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS, ShooterCoefficients.VEL_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);

        setupPaths();

        Pose startingPose = startingPosition == StartingPosition.FAR ? farStartingPose : nearStartingPose;
        if (push) startingPose = pushStartingPose;

        follower.setStartingPose(startingPose);
        Settings.set("pose", startingPose, false);
    }

    @Override
    public void initialize() {
        Settings.set("debug_mode", false, true);
        Settings.set("tabletop_mode", false, true);

        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("starting_position", new OptionPrompt<>("SELECT STARTING POSITION", StartingPosition.FAR, StartingPosition.CLOSE))
                .prompt("cycle", new BooleanPrompt("RUN CYCLE ROUTINE?", false))
                .prompt("pickup_order",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("cycle"))) return null;
                            return new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 0, 1, 2, 3, 4);
                        }
                )
                .prompt("open_gate",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("cycle"))) return null;
                            return new BooleanPrompt("OPEN GATE?", false);
                        }
                )
                .prompt("gate_spike",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("cycle"))) return null;
                            if (Boolean.TRUE.equals(prompter.get("open_gate"))) {
                                return new OptionPrompt<>("AFTER WHICH SPIKE MARK?", 3, 4);
                            }
                            return null;
                        }
                )
                .prompt("push",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("cycle"))) return null;
                            if (prompter.get("starting_position").equals(StartingPosition.FAR)) {
                                return new BooleanPrompt("PUSH OTHER ROBOT?", false);
                            }
                            return null;
                        }
                )
                .onComplete(this::afterPrompts);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void onStart() {
        boolean doCycle = prompter.get("cycle");
        Command autonomousRoutine;

        if (doCycle) {
            autonomousRoutine = (startingPosition == StartingPosition.CLOSE)
                    ? closeCycleRoutine()
                    : farCycleRoutine();
        } else {
            autonomousRoutine = standardRoutine();
        }

        schedule(
                new SequentialCommandGroup(
                        autonomousRoutine,
                        new InstantCommand(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void run() {
        telemetry.update();

        final double inchesToMeters = 39.37;

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d robotPose = new Pose2d(-rotatedPose.getX() / inchesToMeters, -rotatedPose.getY() / inchesToMeters, new Rotation2d(rotatedPose.getHeading() - Math.PI));

        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Voltage", voltageSensor.getVoltage());
        Logger.recordOutput("Reached RPM", shooter.reachedRPM());
        Logger.recordOutput("Reached Angle", shooter.reachedAngle());
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
        Logger.recordOutput("Shooter/Flywheel Error", Math.abs(shooter.getRPM() - shooter.solution.getRPM()));
        Logger.recordOutput("Shooter/Flywheel Target", shooter.getTargetRPM());
        Logger.recordOutput("Shooter/Hood Raw Position", shooter.getRawHoodPosition());
        Logger.recordOutput("Turret/Turret Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
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

    // ---- ROUTINES ----

    private Command standardRoutine() {
        return new SequentialCommandGroup(
                pushRoutine(),
                initialScore(),
                pickupSequence(),
                parkRoutine()
        );
    }

    private Command closeCycleRoutine() {
        return new SequentialCommandGroup(
                initialScore(), // Score first 3 artifacts
                collect(3), // Collect spike 3 and shoot
                returnAndScore(3, false),
                new RepeatCommand(
                        closeCycle(),
                        2
                ), // Cycle from gate 2 times
                collect(4), // Collect spike 4 & 2 and shoot from parking point
                returnAndScore(4, false),
                collect(2),
                returnAndScore(2, true)
        );
    }

    private Command farCycleRoutine() {
        return new SequentialCommandGroup(
                initialScore(), // Score first 3 artifacts
                new RepeatCommand(
                        farCycle(),
                        3
                ), // Cycle from LOADING ZONE 3 times
                parkRoutine() // Park
        );
    }

    private Command closeCycle() {
        return new SequentialCommandGroup(
                // Open gate, collect, and go back to shoot
                new InstantCommand(intake::collect),
                new DeferredCommand(() -> new FollowPathCommand(follower, collectFromGate()), null),
                new WaitCommand(3000),
                new InstantCommand(intake::stop),
                new DeferredCommand(() -> new FollowPathCommand(follower, backFromGateCollection()), null),
                shoot()
        );
    }

    private Command farCycle() {
        return new SequentialCommandGroup(
                // Go to LOADING ZONE, collect, and go back to shoot
                collect(0),
                returnAndScore(0, false)
        );
    }

    private Command pushRoutine() {
        return push ? new FollowPathCommand(follower, pushPath) : new InstantCommand();
    }

    private Command initialScore() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, getBackPath(0)),
                new WaitCommand(2000),
                shoot()
        );
    }

    private Command pickupSequence() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        for (int i = 0; i < pickupOrder.size(); i++) {
            int spike = pickupOrder.get(i);
            boolean isLast = i == (pickupOrder.size() - 1);

            sequence.addCommands(
                    collect(spike),
                    openGate(spike),
                    returnAndScore(spike, isLast)
            );
        }

        return sequence;
    }

    private Command collect(int spike) {
        PathChain[] paths = (startingPosition == StartingPosition.FAR) ? farPaths : nearPaths;
        return new SequentialCommandGroup(
                new InstantCommand(intake::collect),
                new FollowPathCommand(follower, paths[spike - 1]),
                new InstantCommand(intake::stop)
        );
    }

    private Command openGate(int spike) {
        if (spike != gateSpike) return new InstantCommand();

        return new SequentialCommandGroup(
                new FollowPathCommand(follower, spike == 3 ? spike3Open : spike4Open),
                new WaitCommand(100)
        );
    }

    private Command returnAndScore(int spike, boolean isLast) {
        Supplier<PathChain> path = isLast ? this::getFinalPath : () -> getBackPath(spike);
        return new SequentialCommandGroup(
                new DeferredCommand(() -> new FollowPathCommand(follower, path.get()), null),
                shoot()
        );
    }

    private Command parkRoutine() {
        return (startingPosition == StartingPosition.FAR)
                ? new FollowPathCommand(follower, farDriveBackEnd)
                : new InstantCommand();
    }

    private Command shoot() {
        return new ShootCommand(
                shooter, intake, transfer, drive,
                () -> startingPosition == StartingPosition.FAR
        ).asProxy();
    }

    private PathChain getBackPath(int spike) {
        return (startingPosition == StartingPosition.FAR) ? farPathsReturn[spike].get() : nearPathsReturn[spike].get();
    }

    private PathChain getFinalPath() {
        return (startingPosition == StartingPosition.FAR) ? farDriveBack() : nearDriveBackEnd;
    }
}
