package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.opModes.TeleOpApp.ROBOT_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.skeletonarmy.marrow.OpModeManager;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.settings.Settings;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

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
import org.firstinspires.ftc.teamcode.utilities.FollowerManager;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.util.Collection;
import java.util.List;
import java.util.function.Supplier;

@Autonomous(name="Autonomous", preselectTeleOp="TeleOpApp")
public class AutonomousApp extends ComplexOpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(17, 17);

    private final Prompter prompter = new Prompter(this);
    TimerEx matchTime = new TimerEx(30); // 30 second autonomous

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;

    private Pose farStartingPose;
    private Pose nearStartingPose;

    private final PathChain[] farPaths = new PathChain[4];
    private final PathChain[] nearPaths = new PathChain[4];
    private final Supplier<PathChain>[] nearPathsReturn = new Supplier[4];
    private final Supplier<PathChain>[] farPathsReturn = new Supplier[4];
    private PathChain farDriveBackEnd;
    private PathChain nearDriveBackEnd;
    private PathChain spike3Open;
    private PathChain spike4Open;
    private PathChain nearSpike3Open;
    private PathChain nearSpike4Open;
    private PathChain driveToGate;

    private Alliance alliance;
    private boolean endGate;
    private StartingPosition startingPosition;
    private List<Integer> pickupOrder;
    private int gateSpike;

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
                        getRelative(Math.toRadians(154.8622))
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
                                nearDriveBack
                        )
                )
                .setLinearHeadingInterpolation(
                        follower.getHeading(),
                        getRelative(Math.toRadians(180))
                )
                .build();
    }

    public void setupPaths() {
        farStartingPose = getRelative(new Pose(55.61,7.48, Math.toRadians(90)));
        nearStartingPose = getRelative(new Pose(22.56, 119.140000000000000000, Math.toRadians(141.5)));

        Pose spike1End = getRelative(new Pose(16, 9.708060475161995));
        Pose spike2End = getRelative(new Pose(18, 34.76673866090713));
        Pose spike3End = getRelative(new Pose(19, 56));
        Pose spike4End = getRelative(new Pose(19, 83.663));

        Pose openGateEnd = getRelative(new Pose(21, 72));
        Pose spike3GateEnd = getRelative(new Pose(21, 66));
        Pose spike4GateEnd = getRelative(new Pose(21, 83.663));

        farDriveBack = getRelative(new Pose(52, 15.862));
        nearDriveBack = getRelative(new Pose(50, 90));
        gateOpenPose = getRelative(new Pose(14.5721, 58.82221));

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
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(48, 9)),
                                spike1End
                        )
                )
                .setTranslationalConstraint(5)
                .setTValueConstraint(0.7)
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(18, 9.708060475161995))
                        )
                )
                .setTranslationalConstraint(5)
                .setTValueConstraint(0.7)
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(18.5, 19.5)),
                                getRelative(new Pose(16, 19))
                        )
                )
                .setTranslationalConstraint(5)
                .setTValueConstraint(0.7)
                .setTangentHeadingInterpolation()
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
                .build();

        farDriveBackEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                getRelative(new Pose(40, 15.862))
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
                                getRelative(new Pose(51.198, 30.343)),
                                getRelative(new Pose(45.574, 35.113)),
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
                                getRelative(new Pose(50.536, 57.712)),
                                getRelative(new Pose(42.630, 57.712)),
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
                                getRelative(new Pose(46.1919, 82.410)),
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

        nearSpike3Open = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(50.536, 55)),
                                getRelative(new Pose(42.630, 55)),
                                spike3GateEnd
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        nearSpike4Open = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                getRelative(new Pose(13, 86.5)),
                                getRelative(new Pose(22, 79)),
                                spike4GateEnd
                        )
                )
                .setConstantHeadingInterpolation(
                        getRelative(Math.toRadians(180))
                )
                .build();

        driveToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                getRelative(new Pose(23.65442764578834, 68.34557235421167))
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
        pickupOrder = prompter.getOrDefault("pickup_order", List.of());
        gateSpike = prompter.getOrDefault("gate_spike", -1);
        endGate = prompter.getOrDefault("end_near_gate", true);

        Settings.set("alliance", alliance);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.addData("Pickup Order", pickupOrder);

        follower = FollowerManager.createFollower(hardwareMap);

        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);

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
                .prompt("sorted",
                        () -> {
                            if (prompter.get("starting_position").equals(StartingPosition.FAR)) return null;
                            return new BooleanPrompt("RUN VIRTUAL SORTING?", false);
                        }
                )
                .prompt("cycle",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("sorted"))) return null;
                            return new BooleanPrompt("RUN CYCLE ROUTINE?", false);
                        }
                )
                .prompt("pickup_order",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("sorted"))) return null;
                            return new MultiOptionPrompt<>("SELECT ARTIFACT PICKUP ORDER", false, true, 0, 1, 2, 3, 4);
                        }
                )
                .prompt("open_gate",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("sorted"))) return null;
                            if (Boolean.TRUE.equals(prompter.get("cycle"))) return null;
                            return new BooleanPrompt("OPEN GATE?", false);
                        }
                )
                .prompt("gate_spike",
                        () -> {
                            if (Boolean.TRUE.equals(prompter.get("sorted"))) return null;
                            if (Boolean.TRUE.equals(prompter.get("cycle"))) return null;
                            if (Boolean.TRUE.equals(prompter.get("open_gate"))) {
                                return new OptionPrompt<>("AFTER WHICH SPIKE MARK?", 3, 4);
                            }
                            return null;
                        }
                )
                .prompt("end_near_gate", () -> {
                    if (Boolean.TRUE.equals(prompter.get("sorted"))) return null;
                    if (prompter.get("starting_position").equals(StartingPosition.FAR)) return null;
                    return new BooleanPrompt("END NEAR GATE?", true);
                })
                .onComplete(this::afterPrompts);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void onStart() {
        matchTime.start();

        boolean doCycle = prompter.getOrDefault("cycle", false);
        boolean sorted = prompter.getOrDefault("sorted", false);

        Command cycleRoutine = (startingPosition == StartingPosition.CLOSE)
                ? closeCycleRoutine()
                : farCycleRoutine();

        Command autonomousRoutine = doCycle
                ? cycleRoutine
                : (sorted ? sortedRoutine() : standardRoutine());

        schedule(
                new SequentialCommandGroup(
                        autonomousRoutine,
                        new WaitCommand(5000),
                        new InstantCommand(this::requestOpModeStop)
                )
        );
    }

    @Override
    public void run() {
        follower.update();

        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());

        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        shooter.setUpdateFlywheel(distanceFromLaunchZone() < 20);

        telemetry.addData("Turret/Turret Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret/Angle Target (deg)", Math.toDegrees(shooter.wrapped));
        telemetry.addData("Turret/Angle Error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));
        telemetry.addData("CanShootRPM", shooter.getCanShootRPMCalc());
        telemetry.addData("reachedAngle", shooter.reachedAngle());
        telemetry.addData("canShoot", shooter.getCanShoot());
        telemetry.addData("In Zone", isInsideLaunchZone());

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
        Logger.recordOutput("In Zone", isInsideLaunchZone());

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
                parkRoutine()
        );
    }

    private Command sortedRoutine() {
        return new SequentialCommandGroup(
                initialScore(),
                // Detect obelisk
                pickupSequence()
        );
    }

    private Command closeCycleRoutine() {
        return new SequentialCommandGroup(
                initialScore(), // Score first 3 artifacts
                collect(3), // Collect spike 3 and shoot
                returnAndScore(3, false),
                closeCycle(),
                closeCycle(),
                pickupSequence()
        );
    }

    private Command farCycleRoutine() {
        return new SequentialCommandGroup(
                initialScore(), // Score first 3 artifacts
                pickupSequence(),
                repeatIfTime(this::farCycle, 4.0),
                parkRoutine() // Park
        );
    }

    private Command closeCycle() {
        return new SequentialCommandGroup(
                // Open gate, collect, and go back to shoot
                new InstantCommand(intake::collect),
                new DeferredCommand(() -> new FollowPathCommand(follower, collectFromGate()), null),
                new WaitUntilCommand(transfer.threeArtifactsDetected(intake::isCollecting, 250))
                        .withTimeout(2000),
                new InstantCommand(intake::collect),
                new DeferredCommand(() -> new FollowPathCommand(follower, backFromGateCollection()), null),
                new InstantCommand(intake::stop),
                shoot()
        );
    }

    private Command farCycle() {
        return new SequentialCommandGroup(
                // Go to LOADING ZONE, collect, and go back to shoot
                collect(1).withTimeout(3000),
                returnAndScore(1, false)
        );
    }

    private Command initialScore() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, getBackPath(1)),
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
        Supplier<PathChain> path = (isLast && !endGate) ? this::getFinalPath : () -> getBackPath(spike);

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(intake::collect),
                        new DeferredCommand(() -> new FollowPathCommand(follower, path.get()), null),
                        new InstantCommand(intake::stop),
                        shoot()
                )
        );
    }

    private Command parkRoutine() {
        Command parkPath = (startingPosition == StartingPosition.CLOSE)
                ? new FollowPathCommand(follower, driveToGate)
                : new FollowPathCommand(follower, farDriveBackEnd);

        return new ConditionalCommand(
                parkPath,
                new InstantCommand(),
                () -> matchTime.isMoreThan(1) && endGate
        );
    }

    //private boolean isCycle(){
       // boolean bool = false;
       // if (matchTime.isMoreThan(7)) ? bool = true : bool = false;
    //}

    private Command shoot() {
        return new ShootCommand(
                shooter, intake, transfer, drive,
                () -> startingPosition == StartingPosition.FAR
        ).asProxy();
    }

    private PathChain getBackPath(int spike) {
        return (startingPosition == StartingPosition.FAR) ? farPathsReturn[spike - 1].get() : nearPathsReturn[spike - 1].get();
    }

    private PathChain getFinalPath() {
        return (startingPosition == StartingPosition.FAR) ? farDriveBack() : nearDriveBackEnd;
    }

    public boolean isInsideLaunchZone() {
        boolean insideClose = robotZone.isInside(closeLaunchZone);
        boolean insideFar = robotZone.isInside(farLaunchZone);
        return insideClose || insideFar;
    }

    public double distanceFromLaunchZone() {
        return Math.min(robotZone.distanceTo(closeLaunchZone), robotZone.distanceTo(farLaunchZone));
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
