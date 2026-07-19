package org.firstinspires.ftc.teamcode.opModes;

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
import com.skeletonarmy.marrow.settings.Settings;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.commands.GoToArtifactCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.consts.CloseShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.FarShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.LaunchZone;
import org.firstinspires.ftc.teamcode.enums.ShootingPosition;
import org.firstinspires.ftc.teamcode.enums.StartingPosition;
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
    private final TimerEx matchTime = new TimerEx(30); // 30 second autonomous

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;
    private Vision vision;

    private Pose farStartingPose;
    private Pose nearStartingPose;
    private  Pose middleStartingPose;

    private Pose startingPose;

    private final PathChain[] farPaths = new PathChain[4];
    private final PathChain[] nearPaths = new PathChain[4];
    private final Supplier<PathChain>[] nearPathsReturn = new Supplier[4];
    private final Supplier<PathChain>[] farPathsReturn = new Supplier[4];
    private final Supplier<PathChain>[] middlePathsReturn = new Supplier[4];
    private PathChain spike3Open;
    private PathChain spike4Open;
    private PathChain obeliskInitialScorePath;
    private PathChain initialFarPath;
    private PathChain initialMiddlePath;
    private PathChain initialNearPath;
    private PathChain collectSpike4AndOpen;

    private Alliance alliance;
    private StartingPosition startingPosition;
    private List<Integer> pickupOrder;
    private boolean takeSpike;
    private boolean openGate;

    private VoltageSensor voltageSensor;

    private Pose farDriveBack;
    private Pose middleDriveBack;
    private Pose nearDriveBack;
    private double lastLoopTime = 0;

    private BooleanSupplier threeArtifactsDetectedSupplier;

    public PathChain farDriveBack() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                farDriveBack
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(180)))
                                )
                        )
                )
                .setGlobalDeceleration()
                .build();
    }
    public PathChain middleDriveBack() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                middleDriveBack
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1,
                                        HeadingInterpolator.constant(middleDriveBack.getHeading())
                                )
                        )
                )
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

    public PathChain getNearCyclePath() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                getRelative(new Pose(15, 108.5))
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.5,
                                        1,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(162)))
                                )
                        )
                )
                .setGlobalDeceleration()
                .build();
    }

    public PathChain getCollectGate() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                getRelative(new Pose(12, 103))
                        )
                )
                .setConstantHeadingInterpolation(getRelative(Math.toRadians(158)))
                .setGlobalDeceleration()
                .build();
    }

    public void setupPaths() {
        farStartingPose = getRelative(new Pose(79,7.48, Math.toRadians(90)));
        nearStartingPose = getRelative(new Pose(17, 162, Math.toRadians(270)), 192);
        middleStartingPose = getRelative(new Pose(80, 162, Math.toRadians(270)));

        Pose nearSpike1End = getRelative(new Pose(8.5, 11.708060475161995));
        Pose farSpike1End = getRelative(new Pose(9, 7));
        Pose spike2End = getRelative(new Pose(7.5, 60));
        Pose spike3End = getRelative(new Pose(7.5, 81));
        Pose spike4End = getRelative(new Pose(10.252, 105.570));

        Pose openGateEnd = getRelative(new Pose(15, 124));

        farDriveBack = getRelative(new Pose(71, 23.67));
        middleDriveBack = (prompter.getOrDefault("shooting_position", ShootingPosition.CLOSE) == ShootingPosition.CLOSE)
                ? getRelative(new Pose(80, 105, Math.toRadians(200)))
                : getRelative(new Pose(80, 65, Math.toRadians(180)));
        nearDriveBack = getRelative(new Pose(65.5, 119));

        nearPathsReturn[0] = this::nearDriveBack;
        nearPathsReturn[1] = this::nearDriveBack;
        nearPathsReturn[2] = () -> follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower.getPose(),
                                getRelative(new Pose(64.458, 87.840)),
                                getRelative(new Pose(43.607, 77.298)),
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

        middlePathsReturn[0] = this::middleDriveBack;
        middlePathsReturn[1] = this::middleDriveBack;
        middlePathsReturn[2] = this::middleDriveBack;
        middlePathsReturn[3] = this::middleDriveBack;

        initialFarPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                farDriveBack
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        initialMiddlePath = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(40, 140)),
                                middleDriveBack
                        )
                )
                .setLinearHeadingInterpolation(
                        getRelative(Math.toRadians(270)),
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        initialNearPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(53.138, 134.795))
                        )
                )
                .setLinearHeadingInterpolation(
                        getRelative(Math.toRadians(270)),
                        getRelative(Math.toRadians(180))
                )
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
                                getRelative(new Pose(70, 60)),
                                getRelative(new Pose(60, 60)),
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
                                getRelative(new Pose(70, 81)),
                                getRelative(new Pose(60, 81)),
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
                                getRelative(new Pose(70, 105.570)),
                                getRelative(new Pose(60, 105.570)),
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
                                getRelative(new Pose(8.452, 47.408)),
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
                                getRelative(new Pose(56, 60)),
                                getRelative(new Pose(50, 60)),
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
                                getRelative(new Pose(56, 81)),
                                getRelative(new Pose(50, 81)),
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
                                getRelative( new Pose(56.061, 105.570)),
                                spike4End
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .setGlobalDeceleration()
                .build();

        collectSpike4AndOpen = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(54, 108)),
                                getRelative(new Pose(14, 108)),
                                getRelative(new Pose(12, 110))
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(180)))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(160)))
                                )
                        )
                )
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
                                getRelative(new Pose(25, 76.111)),
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
                                        0.1,
                                        HeadingInterpolator.constant(getRelative(Math.toRadians(270)))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.1,
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
        takeSpike = prompter.getOrDefault("take_spike", true);
        openGate = prompter.getOrDefault("open_gate", true);

        Settings.set("alliance", alliance);

        follower = FollowerManager.createFollower(hardwareMap);

        IShooterCalculator shooterCalcClose = new ShooterCalculator(new CloseShooterCoefficients());
        IShooterCalculator shooterCalcFar = new ShooterCalculator(new FarShooterCoefficients());
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalcClose, shooterCalcFar, alliance);

        if (startingPosition == StartingPosition.FAR) {
            shooter.setSOTMEnabled(false);
            shooter.setZoneCalculator(LaunchZone.FAR);
        }

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);

        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.DETECTION_PIPELINE);

        setupPaths();
        if (startingPosition == StartingPosition.FAR) {
            startingPose = farStartingPose;
        } else if (startingPosition == StartingPosition.MIDDLE) {
            startingPose = middleStartingPose;
        } else {
            startingPose = nearStartingPose;
        }

        follower.setPose(startingPose);

        threeArtifactsDetectedSupplier = transfer.threeArtifactsDetected(() -> true, 100);
    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        Settings.set("debug_mode", false, true);
        Settings.set("tabletop_mode", false, true);

        prompter.prompt("alliance", new OptionPrompt<>("SELECT ALLIANCE", Alliance.RED, Alliance.BLUE))
                .prompt("starting_position", new OptionPrompt<>("SELECT STARTING POSITION", StartingPosition.FAR, StartingPosition.CLOSE, StartingPosition.MIDDLE))
                .prompt("shooting_position", new OptionPrompt<>("SHOOTING POSITION", ShootingPosition.CLOSE, ShootingPosition.FAR))
                    .showIf("starting_position", StartingPosition.MIDDLE)
                .prompt("cycle", new BooleanPrompt("RUN CYCLE ROUTINE?", false))
                .prompt("take_spike", new BooleanPrompt("TAKE SPIKE 3?", true))
                    .showIf("starting_position", StartingPosition.CLOSE)
                    .showIf("cycle", true)
                .prompt("open_gate", new BooleanPrompt("OPEN GATE?", true))
                    .showIf("starting_position", StartingPosition.CLOSE)
                    .showIf("cycle", true)
                .prompt("take_spike", new BooleanPrompt("TAKE SPIKE 3?", true))
                    .showIf("starting_position", StartingPosition.CLOSE)
                    .showIf("cycle", true)
                .prompt("pickup_order", new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 0, 1, 2, 3, 4))
                    .showIf("cycle", false)
                    .or("starting_position", StartingPosition.FAR)
                    .or("starting_position", StartingPosition.MIDDLE)
                .showSummary()
                .onComplete(this::afterPrompts);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void onStart() {
        matchTime.start();

        boolean doCycle = prompter.getOrDefault("cycle", false);

        Command cycleRoutine;
        if (startingPosition == StartingPosition.CLOSE) {
            cycleRoutine = closeCycleRoutine();
        } else if (startingPosition == StartingPosition.MIDDLE) {
            cycleRoutine = middleCycleRoutine();
        } else {
            cycleRoutine = farCycleRoutine();
        }

        Command autonomousRoutine = doCycle
                ? cycleRoutine
                : standardRoutine();

        schedule(
                new SequentialCommandGroup(
                        autonomousRoutine,
                        new WaitCommand(1000),
                        new InstantCommand(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void run() {
        double currentTime = matchTime.getElapsed();
        double loopTimeMs = (currentTime - lastLoopTime) * 1000.0;
        lastLoopTime = currentTime;

        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        shooter.setUpdateFlywheel(drive.distanceFromLaunchZone() < 40);

        telemetry.addData("loop times", loopTimeMs);
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
//        telemetry.addData("detected artifacts", threeArtifactsDetectedSupplier.getAsBoolean());

        final double inchesToMeters = 39.37;

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d robotPose = new Pose2d(-rotatedPose.getX() / inchesToMeters, -rotatedPose.getY() / inchesToMeters, new Rotation2d(rotatedPose.getHeading() - Math.PI));

        Logger.recordOutput("Loop times", loopTimeMs);
        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Voltage", voltage);
        Logger.recordOutput("Shooter/Flywheel RPM", shooter.filteredRPM);
        Logger.recordOutput("Shooter/Flywheel Error", Math.abs(shooter.filteredRPM - shooter.getTargetRPM()));
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
            return originalPose.mirror(GoalPositions.FIELD_LENGTH);
        }

        return originalPose;
    }

    private Pose getRelative(Pose originalPose, double fieldLength) {
        if (alliance == Alliance.RED) {
            return originalPose.mirror(fieldLength);
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
                startingPosition == StartingPosition.CLOSE ? nearInitialScore() : farInitialScore(),
                pickupSequence(),
                driveForward()
        );
    }

    private Command closeCycleRoutine() {
        return new SequentialCommandGroup(
                nearInitialScore(), // Score first 3 artifacts
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::collect),
                                new FollowPathCommand(follower, collectSpike4AndOpen)
                                        .interruptOn(() -> follower.getCurrentTValue() > 0.5 && follower.getVelocity().getMagnitude() < 1.0),
                                new InstantCommand(intake::stop)
                        ),
                        collect(4).interruptOn(threeArtifactsDetectedSupplier),
                        () -> openGate
                ),
                returnAndScore(4),
                closeCycle(),
                closeCycle(),
                closeCycle(),
                closeCycle(),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                collect(3).interruptOn(threeArtifactsDetectedSupplier),
                                returnAndScore(4)
                                        .interruptOn(() -> matchTime.getRemaining() < 0.2)
                        ),
                        closeCycle(),
                        () -> takeSpike
                ),
                driveForward()
        );
    }

    private Command farCycleRoutine() {
        return new SequentialCommandGroup(
                farInitialScore(), // Score first 3 artifacts
                pickupSequence(),
                collect(1)
                        .withTimeout(2000),
                returnAndScore(1),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> matchTime.isLessThan(0.2)), // Cancel if no time to park last minute
                        repeatIfTime(this::farCycle, 0.0)
                ),
                driveForward()
        );
    }

    private Command middleCycleRoutine() {
        return new SequentialCommandGroup(
                middleInitialScore(),
                pickupSequence(),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> matchTime.isLessThan(0.2)), // Cancel if no time to park last minute
                        repeatIfTime(this::farCycle, 0.0)
                ),
                driveForward()
                // console.log("Roem Samsh need to make new instant coffee for me.,"
        );
    //    "razi" "roem samsh"
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
        return new SequentialCommandGroup(
                new InstantCommand(intake::collect),
                new DeferredCommand(() -> new FollowPathCommand(follower, getNearCyclePath()).withTimeout(1000), null),
                new WaitCommand(400),
                new DeferredCommand(() -> new FollowPathCommand(follower, getCollectGate()), null)
                        .interruptOn(threeArtifactsDetectedSupplier)
                        .withTimeout(1500),
                followAndShoot(this::nearDriveBack)
        );
    }

    private Command farCycle() {
        return new SequentialCommandGroup(
                // Go to LOADING ZONE, collect, and go back to shoot
                new InstantCommand(intake::collect),
                new GoToArtifactCommand(follower, vision, alliance)
                        .interruptOn(threeArtifactsDetectedSupplier),
                returnAndScore(1)
        );
    }

    private Command nearInitialScore() {
        return new ParallelCommandGroup(
//                new InstantCommand(() -> shooter.setSOTMEnabled(true)),
                new FollowPathCommand(follower, initialNearPath),
                shoot()
//                new InstantCommand(() -> shooter.setSOTMEnabled(false))
        );
    }

    private Command farInitialScore() {
        return new ParallelCommandGroup(
                new DeferredCommand(() -> new FollowPathCommand(follower, initialFarPath), null),
                new SequentialCommandGroup(
                        shoot(),
                        new InstantCommand(follower::breakFollowing)
                )
        );
    }

    private Command middleInitialScore() {
        return new ParallelCommandGroup(
            new DeferredCommand(() -> new FollowPathCommand(follower, initialMiddlePath), null),
            new SequentialCommandGroup(
                shoot(),
                new InstantCommand(follower::breakFollowing))
        );
    }

    private Command pickupSequence() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        for (int i = 0; i < pickupOrder.size(); i++) {
            int spike = pickupOrder.get(i);

            sequence.addCommands(
                    collect(spike).interruptOn(threeArtifactsDetectedSupplier),
                    returnAndScore(spike)
            );
        }

        return sequence;
    }

    private Command collect(int spike) {
        PathChain[] paths = (startingPosition == StartingPosition.FAR) ? farPaths : nearPaths;
        PathChain path = paths[spike - 1];
        return new SequentialCommandGroup(
                new InstantCommand(intake::collect),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> spike != 1 && follower.getCurrentTValue() > 0.5 && follower.getVelocity().getMagnitude() < 30.0),
                                new InstantCommand(follower::breakFollowing)
                        )
                ),
                new InstantCommand(intake::stop)
        );
    }

    private Command returnAndScore(int spike) {
        return new SequentialCommandGroup(
                new InstantCommand(intake::collect),
                followAndShoot(() -> getBackPath(spike))
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
        if (startingPosition == StartingPosition.FAR)
            return new SequentialCommandGroup(
                    new DeferredCommand(() -> new FollowPathCommand(follower, pathSupplier.get()), null),
                    new WaitCommand(200),
                    shootCommand,
                    new InstantCommand(follower::breakFollowing)
            );

        return new ParallelCommandGroup(
                new DeferredCommand(() -> new FollowPathCommand(follower, pathSupplier.get()), null),
                new SequentialCommandGroup(
                        new WaitUntilCommand(drive::isInsideLaunchZonePredictive),
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
                new WaitCommand(100),
                new InstantCommand(intake::collect),
                transfer.kick(),
                new InstantCommand(() -> {
                    intake.stop();
                    transfer.block();
                })
        );
    }

    private PathChain getBackPath(int spike) {
        if (startingPosition == StartingPosition.FAR) {
            return farPathsReturn[spike - 1].get();
        } else if (startingPosition == StartingPosition.MIDDLE) {
            return middlePathsReturn[spike - 1].get();
        } else {
            return nearPathsReturn[spike - 1].get();
        }
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
