package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.SLOW_SHOOTING_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.commands.CloseCycleCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.consts.CloseShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.FarShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.ArtifactPattern;
import org.firstinspires.ftc.teamcode.enums.StartingPosition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.firstinspires.ftc.teamcode.utilities.FollowerManager;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOpApp")
public class AutonomousApp extends ComplexOpMode {
    private final Prompter prompter = new Prompter(this);
    TimerEx matchTime = new TimerEx(30); // 30 second autonomous

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;
    private Vision vision;

    private Pose farStartingPose;
    private Pose nearStartingPose;

    private final PathChain[] farPaths = new PathChain[4];
    private final PathChain[] nearPaths = new PathChain[4];
    private final Supplier<PathChain>[] nearPathsReturn = new Supplier[4];
    private final Supplier<PathChain>[] farPathsReturn = new Supplier[4];
    private PathChain nearDriveBackEnd;
    private PathChain sortEnd;
    private PathChain spike3Open;
    private PathChain spike4Open;
    private PathChain obeliskInitialScorePath;
    private PathChain initialFarPath;
    private PathChain initialNearPath;

    private Alliance alliance;
    private StartingPosition startingPosition;
    private List<Integer> pickupOrder;
    private int gateSpike;

    private VoltageSensor voltageSensor;

    private Pose farDriveBack;
    private Pose nearDriveBack;
    private Pose sortingPose;

    private boolean isSorting = false;
    private ArtifactPattern detectedPattern = ArtifactPattern.PPG; // fallback default

    public PathChain farDriveBack() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                farDriveBack
                        )
                )
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
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
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration()
                .build();
    }

    public PathChain goSort() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                getRelative(new Pose(40, 82.348)),
                                sortingPose
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        getRelative(Math.toRadians(141.5))
                )
                .setGlobalDeceleration()
                .build();
    }

    public PathChain sortAgain() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                sortingPose
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        getRelative(Math.toRadians(141.5))
                )
                .setTValueConstraint(0.8)
                .setTranslationalConstraint(3)
                .setVelocityConstraint(3)
                .setGlobalDeceleration()
                .build();
    }

    public PathChain collectSorted() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                nearStartingPose
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(141.5))
                )
                .setGlobalDeceleration()
                .build();
    }

    public PathChain getFarCyclePath() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                getRelative(new Pose(48, 9)),
                                getRelative(new Pose(12, 8))
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .addPath(
                        new BezierLine(
                                getRelative(new Pose(12, 8)),
                                getRelative(new Pose(11, 35))
                        )
                )
                .setTranslationalConstraint(5)
                .setTValueConstraint(0.7)
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();
    }

    public void setupPaths() {
        farStartingPose = getRelative(new Pose(55,7.48, Math.toRadians(90)));
        nearStartingPose = getRelative(new Pose(15, 112, Math.toRadians(270)));

        Pose nearSpike1End = getRelative(new Pose(10, 9.708060475161995));
        Pose farSpike1End = getRelative(new Pose(11.5555, 8));
        Pose spike2End = getRelative(new Pose(12, 34.76673866090713));
        Pose spike3End = getRelative(new Pose(12, 56));
        Pose spike4End = getRelative(new Pose(19, 83.663));

        Pose openGateEnd = getRelative(new Pose(21, 72));

        farDriveBack = getRelative(new Pose(48, 17.9901));
        nearDriveBack = getRelative(new Pose(50, 90));
        sortingPose = getRelative(new Pose(30, 113));

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
                .setGlobalDeceleration()
                .build();
        nearPathsReturn[3] = this::nearDriveBack;

        farPathsReturn[0] = this::farDriveBack;
        farPathsReturn[1] = this::farDriveBack;
        farPathsReturn[2] = this::farDriveBack;
        farPathsReturn[3] = this::farDriveBack;

        initialFarPath = follower
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
                .setGlobalDeceleration()
                .build();

        initialNearPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                nearDriveBack
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(0))
                )
                .setReversed()
                .setGlobalDeceleration()
                .build();

        farPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(48, 9)),
                                farSpike1End
                        )
                )
                .setTranslationalConstraint(5)
                .setTValueConstraint(0.7)
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        farPaths[1] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(54.152, 39.94)),
                                getRelative(new Pose(40.674, 34.313)),
                                spike2End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        farPaths[2] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(56.213, 63.913)),
                                getRelative(new Pose(51.6035, 58.214)),
                                spike3End

                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        farPaths[3] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(55.843, 93.284)),
                                getRelative(new Pose(51.780, 83.231)),
                                spike4End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        nearPaths[0] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(9.952, 45.408)),
                                nearSpike1End
                        )
                )
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();

        nearPaths[1] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(51.198, 30.343)),
                                getRelative(new Pose(45.574, 35.113)),
                                spike2End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        nearPaths[2] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(50.536, 57.712)),
                                getRelative(new Pose(42.630, 57.712)),
                                spike3End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        nearPaths[3] = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(46.1919, 82.410)),
                                spike4End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
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
                .setGlobalDeceleration()
                .build();

        sortEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(43, 123))
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setGlobalDeceleration()
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
                .setGlobalDeceleration()
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
                .setGlobalDeceleration()
                .build();

        obeliskInitialScorePath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                nearDriveBack
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.8,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(70)))
                                )
                        )
                )
                .setGlobalDeceleration()
                .build();
    }

    public void afterPrompts() {
        alliance = prompter.get("alliance");
        startingPosition = prompter.get("starting_position");
        pickupOrder = prompter.getOrDefault("pickup_order", List.of());
        gateSpike = prompter.getOrDefault("gate_spike", -1);

        Settings.set("alliance", alliance);

        follower = FollowerManager.createFollower(hardwareMap);

        IShooterCalculator shooterCalcClose = new ShooterCalculator(new CloseShooterCoefficients());
        IShooterCalculator shooterCalcFar = new ShooterCalculator(new FarShooterCoefficients());
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalcClose, shooterCalcFar, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.OBELISK_PIPELINE);

        setupPaths();

        Pose startingPose = startingPosition == StartingPosition.FAR ? farStartingPose : nearStartingPose;
        follower.setPose(startingPose);
    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        Settings.set("debug_mode", false, true);
        Settings.set("tabletop_mode", false, true);

        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("starting_position", new OptionPrompt<>("SELECT STARTING POSITION", StartingPosition.FAR, StartingPosition.CLOSE))
                .prompt("delay", new ValuePrompt<>("SELECT DELAY", Double.class, 0.0, 5.0, 0.0, 0.5))
                .prompt("sorted", new BooleanPrompt("RUN VIRTUAL SORTING?", false))
                    .showIf("starting_position", StartingPosition.CLOSE)
                .prompt("cycle", new BooleanPrompt("RUN CYCLE ROUTINE?", false))
                    .showIf("sorted", false)
                    .or("starting_position", StartingPosition.FAR)
                .prompt("pickup_order", new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 0, 1, 2, 3, 4))
                    .showIf("sorted", false)
                    .or("starting_position", StartingPosition.FAR)
                    .showIf("cycle", false)
                    .or("starting_position", StartingPosition.FAR)
                .prompt("open_gate", new BooleanPrompt("OPEN GATE?", false))
                    .showIf("sorted", false)
                    .showIf("cycle", false)
                .prompt("gate_spike", new OptionPrompt<>("AFTER WHICH SPIKE MARK?", 3, 4))
                    .showIf("sorted", false)
                    .showIf("cycle", false)
                    .showIf("open_gate", true)
                .showSummary()
                .onComplete(this::afterPrompts);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void onStart() {
        matchTime.start();

        boolean doCycle = prompter.getOrDefault("cycle", false);
        boolean sorted = prompter.getOrDefault("sorted", false);
        double delay = prompter.get("delay");

        Command cycleRoutine = (startingPosition == StartingPosition.CLOSE)
                ? closeCycleRoutine()
                : farCycleRoutine();

        Command autonomousRoutine = doCycle
                ? cycleRoutine
                : (sorted ? sortedRoutine() : standardRoutine());

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand((long)(delay * 1000)),
                        autonomousRoutine,
                        new WaitCommand(5000),
                        new InstantCommand(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void run() {
        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        if (!isSorting) shooter.setUpdateFlywheel(drive.distanceFromLaunchZone() < 40);

        telemetry.addData("Robot Pedro X", follower.getPose().getX());
        telemetry.addData("Robot Pedro Y", follower.getPose().getY());
        telemetry.addData("Robot Pedro Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret/Turret Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret/Angle Target (deg)", Math.toDegrees(shooter.wrapped));
        telemetry.addData("Turret/Angle Error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));
        telemetry.addData("CanShootRPM", shooter.getCanShootRPMCalc());
        telemetry.addData("reachedAngle", shooter.reachedAngle());
        telemetry.addData("canShoot", shooter.getCanShoot());
        telemetry.addData("In Zone", drive.isInsideLaunchZonePredictive());

        final double inchesToMeters = 39.37;

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d robotPose = new Pose2d(-rotatedPose.getX() / inchesToMeters, -rotatedPose.getY() / inchesToMeters, new Rotation2d(rotatedPose.getHeading() - Math.PI));

        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Voltage", voltage);
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.getRPM());
        Logger.recordOutput("Shooter/Flywheel Error", Math.abs(shooter.getRPM() - shooter.getTargetRPM()));
        Logger.recordOutput("Shooter/Flywheel Target", shooter.getTargetRPM());
        Logger.recordOutput("Shooter/Hood Raw Position", shooter.getRawHoodPosition());
        Logger.recordOutput("Turret/Turret Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        Logger.recordOutput("Turret/Angle Target (deg)", Math.toDegrees(shooter.wrapped));
        Logger.recordOutput("Turret/Angle Error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));
        Logger.recordOutput("CanShootRPM", shooter.getCanShootRPMCalc());
        Logger.recordOutput("reachedAngle", shooter.reachedAngle());
        Logger.recordOutput("canShoot", shooter.getCanShoot());
        Logger.recordOutput("In Zone", drive.isInsideLaunchZonePredictive());

        telemetry.addData("Time remaining", matchTime.getRemaining());
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

    // ---- ROUTINES ----

    private Command standardRoutine() {
        return new SequentialCommandGroup(
                initialScore(),
                pickupSequence(),
                driveForward()
        );
    }

    private Command sortedRoutine() {
        gateSpike = 4;

        return new SequentialCommandGroup(
                new FollowPathCommand(follower, obeliskInitialScorePath)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ParallelDeadlineGroup(
                                                shoot(),
                                                detectObelisk().withTimeout(2000)
                                        )
                                )
                        ),

                new DeferredCommand(() -> new SequentialCommandGroup(
                        collectSortAndScore(4, detectedPattern),
                        collectSortAndScore(3, detectedPattern),
                        collectSortAndScore(2, detectedPattern)
                ), null)
        );
    }

    private Command closeCycleRoutine() {
        return new SequentialCommandGroup(
                initialScore(), // Score first 3 artifacts
                collect(3), // Collect spike 3 and shoot
                returnAndScore(3, false),
                closeCycle(),
                closeCycle(),
                closeCycle(),
                collect(4),
                returnAndScore(4, false),
                driveForward()
        );
    }

    private Command farCycleRoutine() {
        return new SequentialCommandGroup(
                initialScore(), // Score first 3 artifacts
                pickupSequence(),
                collect(1)
                        .withTimeout(3000),
                returnAndScore(1, false),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> matchTime.isLessThan(0.2)), // Cancel if no time to park last minute
                        repeatIfTime(this::farCycle, 0.0)
                ),
                driveForward()
        );
    }

    private Command driveForward() {
        // Move forward at max speed
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    follower.startTeleOpDrive();
                    follower.setTeleOpDrive(1, 0, 0, true);
                }),
                new WaitCommand(200),
                new InstantCommand(this::requestOpModeStop)
        );
    }

    private Command closeCycle() {
        return new CloseCycleCommand(follower, intake, transfer, shooter, drive, alliance).asProxy();
    }

    private Command farCycle() {
        PathChain path = pickupOrder.contains(2) ? getFarCyclePath() : farPaths[0];

        return new SequentialCommandGroup(
                // Go to LOADING ZONE, collect, and go back to shoot
                new InstantCommand(intake::collect),
                new FollowPathCommand(follower, path)
                        .withTimeout(2500)
                        .interruptOn(artifactDetected()),
                returnAndScore(1, false)
        );
    }

    private Command initialScore() {
        PathChain path = (startingPosition == StartingPosition.FAR) ? initialFarPath : initialNearPath;

        return new ParallelCommandGroup(
                new FollowPathCommand(follower, path),
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
                new InstantCommand(() -> {
                    // Drive into the gate to make sure it stays open
                    follower.startTeleOpDrive();
                    follower.setTeleOpDrive(0.5, 0, 0, true);
                }),
                new WaitCommand(500)
        );
    }

    private Command returnAndScore(int spike, boolean isLast) {
        Supplier<PathChain> path = isLast ? this::getFinalPath : () -> getBackPath(spike);

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(intake::collect),
                        followAndShoot(path)
                )
        );
    }

    private Command shoot() {
        // TODO: If no ball, don't shoot
        return new ShootCommand(
                shooter, intake, transfer, drive
        ).asProxy();
    }

    private Command followAndShoot(Supplier<PathChain> pathSupplier) {
        return followAndShoot(pathSupplier, shoot());
    }

    private Command followAndShoot(Supplier<PathChain> pathSupplier, Command shootCommand) {
        return new ParallelCommandGroup(
                new DeferredCommand(() -> new FollowPathCommand(follower, pathSupplier.get()), null),
                new SequentialCommandGroup(
                        new WaitUntilCommand(drive::isInsideLaunchZonePredictive),
                        new InstantCommand(intake::stop),
                        shootCommand,
                        new InstantCommand(follower::breakFollowing)
                )
        );
    }

    private Command shootWithKicker() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intake.stop();
                    transfer.release();
                }),
                transfer.kick(),
                new InstantCommand(transfer::block)
        );
    }

    private BooleanSupplier artifactDetected() {
        TimerEx debounce = new TimerEx(0.2);
        return () -> {
            if (transfer.isArtifactDetected() && transfer.isArtifactInIntake()) {
                if (!debounce.isOn()) debounce.restart();
                return debounce.isDone();
            } else {
                if (debounce.isOn()) debounce.pause();
                debounce.restart();
                debounce.pause();
                return false;
            }
        };
    }

    private PathChain getBackPath(int spike) {
        return (startingPosition == StartingPosition.FAR) ? farPathsReturn[spike - 1].get() : nearPathsReturn[spike - 1].get();
    }

    private PathChain getFinalPath() {
        return (startingPosition == StartingPosition.FAR) ? farDriveBack() : nearDriveBackEnd;
    }

    private Command collectSortAndScore(int spike, ArtifactPattern target) {
        ArtifactPattern collectedPattern = ArtifactPattern.fromSpike(spike);
        int sorts = collectedPattern.sortsTo(target);
        boolean isLast = spike == 2;

        Command shootCommand = new ShootCommand(
                shooter, intake, transfer, drive,
                SLOW_SHOOTING_POWER
        );

        Command shootWhileDriving =
                new ParallelCommandGroup(
                    new DeferredCommand(() -> new FollowPathCommand(follower,
                            isLast ? sortEnd : getBackPath(1)
                    ), null),
                    shootCommand.asProxy()
                );

        Command driveThenShoot = new SequentialCommandGroup(
                new InstantCommand(intake::collect),
                followAndShoot(
                        isLast ? this::getFinalPath : () -> getBackPath(spike),
                        shootCommand.asProxy()
                )
        );

        return new SequentialCommandGroup(
                collect(spike),
                openGate(spike),
                sortNTimes(sorts),
                sorts > 0 ? shootWhileDriving : driveThenShoot
        );
    }

    private Command sortNTimes(int n) {
        return new DeferredCommand(() -> {
            SequentialCommandGroup seq = new SequentialCommandGroup();
            if (n == 0) return seq;

            seq.addCommands(
                    new InstantCommand(() -> {
                        isSorting = true;
                        shooter.setUpdateFlywheel(false);
                        shooter.setVerticalManualMode(true);
                        shooter.setHorizontalManualMode(true);

                        shooter.setRPM(600);
                        shooter.setVerticalAngle(Math.toRadians(90));
                        shooter.setHorizontalAngle(Math.toRadians(0));
                    })
            );

            seq.addCommands(n == 1 ? goSortOnce() : goSortTwice());

            seq.addCommands(
                    new InstantCommand(() -> {
                        isSorting = false;
                        shooter.setUpdateFlywheel(true);
                        shooter.setVerticalManualMode(false);
                        shooter.setHorizontalManualMode(false);
                    })
            );

            return seq;
        }, null);
    }

    private Command goSortOnce() {
        return new SequentialCommandGroup(
                new DeferredCommand(() -> new FollowPathCommand(follower, goSort()), null),
                shootWithKicker(),
                collectSortedArtifact()
        );
    }

    private Command goSortTwice() {
        return new SequentialCommandGroup(
                goSortOnce(),
                new ParallelCommandGroup(
                        new DeferredCommand(() -> new FollowPathCommand(follower, sortAgain()).withTimeout(300), null),
                        shootWithKicker()
                ),
                collectSortedArtifact()
        );
    }

    private Command collectSortedArtifact() {
        return new SequentialCommandGroup(
                new InstantCommand(intake::collect),
                new WaitCommand(1000),
                new DeferredCommand(() -> new FollowPathCommand(follower, collectSorted()), null)
                        .withTimeout(1000)
        );
    }

    private Command detectObelisk() {
        return new WaitUntilCommand(() -> {
            ArtifactPattern p = vision.detectPattern();
            if (p != null) {
                detectedPattern = p;
                return true;
            }
            return false;
        });
    }

    /**
     * Repeatedly schedules a command as long as there is enough time remaining.
     * @param cycleSupplier A factory that returns a NEW instance of the cycle command.
     * @param threshold The minimum seconds remaining required to START a new cycle.
     */
    private Command repeatIfTime(Supplier<Command> cycleSupplier, double threshold) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        cycleSupplier.get(),
                        new DeferredCommand(() -> repeatIfTime(cycleSupplier, threshold), null)
                ),
                new InstantCommand(),
                () -> matchTime.isMoreThan(threshold)
        );
    }
}
