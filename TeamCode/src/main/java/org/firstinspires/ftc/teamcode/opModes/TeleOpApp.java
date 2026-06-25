package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.ftc.PoseConverter.poseToPose2D;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;
import static org.firstinspires.ftc.teamcode.config.VisionConfig.LIMELIGHT_NAME;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.settings.Settings;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.config.KalmanConfig;
import org.firstinspires.ftc.teamcode.consts.CloseShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.FarShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.LaunchZone;
import org.firstinspires.ftc.teamcode.opModes.tests.EpochTimestampSyncOpMode2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.utilities.CameraUtil;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.firstinspires.ftc.teamcode.utilities.FusionLocalizer;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

@TeleOp
public class TeleOpApp extends ComplexOpMode {
    public static final double INCHES_TO_METERS = 39.37;
    public static final double ROBOT_WIDTH = 16.53; // Side-to-side
    public static final double ROBOT_LENGTH = 14.96; // Front-to-back
    public static final double X_OFFSET = ROBOT_LENGTH / 2.0;
    public static final double Y_OFFSET = ROBOT_WIDTH / 2.0;
    private static final double METERS_TO_INCHES = 39.37;

    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(ROBOT_LENGTH, ROBOT_WIDTH); // Length maps to X-axis and width maps to Y-axis relative to 0° heading
    private final PolygonZone futureRobotZone = new PolygonZone(ROBOT_LENGTH, ROBOT_WIDTH); // Length maps to X-axis and width maps to Y-axis relative to 0° heading
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Drive drive;
    private Kickstand kickstand;
    private Vision vision;
    private Limelight3A limelight;

    private VoltageSensor voltageSensor;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private boolean debugMode;
    private boolean tabletopMode;
    private Alliance alliance;

    private TimerEx matchTime;
    private TimerEx overrideTimer;

    private boolean isOverrideActive = false;

    private double lastLoopTime = 0;
    private int loopCount = 0;
    private long nanoTimeToEpochMillisOffset = 0;

    AprilTagPipeline aprilTagPipeline;
    private EpochTimestampSyncOpMode2.TimestampedPinpoint pinpoint;

    private double sizeVarianceX;
    private double sizeVarianceY;
    private double sizeVarianceAngle;
    public static boolean testings = true;

    Pose pinpointPose;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        matchTime = new TimerEx(120);
        overrideTimer = new TimerEx(1);

//        debugMode = Settings.get("debug_mode", false);
        debugMode = true;
        tabletopMode = Settings.get("tabletop_mode", false);
        alliance = Settings.get("alliance", Alliance.RED);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setMaxPower(1);

        Pose startPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        follower.setPose(Settings.get("pose", startPose));

        if (debugMode) follower.setPose(startPose);

        // 1. Calculate the real-time synchronization offset between clocks
        long epochMillisSample = System.currentTimeMillis();
        long nanoTimeSample = System.nanoTime();

        // This converts any System.nanoTime() reading directly into an Epoch Millisecond timestamp
        nanoTimeToEpochMillisOffset = epochMillisSample - (nanoTimeSample / 1_000_000);
        I2cDeviceSynchSimple rawI2cClient = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint").getDeviceClient();
        pinpoint = new EpochTimestampSyncOpMode2.TimestampedPinpoint(rawI2cClient);

        IShooterCalculator shooterCalcClose = new ShooterCalculator(new CloseShooterCoefficients());
        IShooterCalculator shooterCalcFar = new ShooterCalculator(new FarShooterCoefficients());
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalcClose, shooterCalcFar, alliance);

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        drive = new Drive(follower, alliance);
        kickstand = new Kickstand(hardwareMap);
        //vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.APRILTAG_PIPELINE);

        //vision.addRelocalizationListener(drive::holdPoint); // Hold the new pose after relocalizing

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        aprilTagPipeline = new AprilTagPipeline();
        CameraUtil.configureWebcam(aprilTagPipeline, hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(intake::collect, intake));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(intake::release, intake, transfer));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(new InstantCommand(() -> {
                    if (isShootingAllowed()) {
                        schedule(new ShootCommand(shooter, intake, transfer, drive));
                    } else {
                        gamepad1.rumble(300);
                        isOverrideActive = true;
                        overrideTimer.restart();
                    }
                }));

        // Long press: fire immediately when entering zone
        new Trigger(() -> gamepadEx1.isDown(GamepadKeys.Button.CROSS) && isInsideLaunchZonePredictive() && shooter.getCurrentCommand() == null)
                .whenActive(new ScheduleCommand(new ShootCommand(shooter, intake, transfer, drive)));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(transfer.kick());

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .and(new Trigger(this::isShootingAllowed))
                .whileActiveContinuous(new ShootCommand(shooter, intake, transfer, drive));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getVerticalManualMode()) shooter.setHoodPosition(shooter.getRawHoodPosition() + 0.01);
                            else shooter.setVerticalOffset(shooter.getVerticalOffset() + 0.05);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getVerticalManualMode()) shooter.setHoodPosition(shooter.getRawHoodPosition() - 0.01);
                            else shooter.setVerticalOffset(shooter.getVerticalOffset() - 0.05);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getHorizontalManualMode()) shooter.setHorizontalAngle(shooter.getTurretAngle(AngleUnit.RADIANS) + 0.2);
                            else shooter.setHorizontalOffset(shooter.getHorizontalOffset() + 0.01);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(
                        new InstantCommand(() -> {
                            if (shooter.getHorizontalManualMode()) shooter.setHorizontalAngle(shooter.getTurretAngle(AngleUnit.RADIANS) - 0.2);
                            else shooter.setHorizontalOffset(shooter.getHorizontalOffset() - 0.01);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SHARE)
                .toggleWhenPressed(
                    new InstantCommand(() -> {
                        shooter.setHorizontalManualMode(true);
                        shooter.setVerticalManualMode(true);
                    }),
                    new InstantCommand(() -> {
                        shooter.setHorizontalManualMode(false);
                        shooter.setVerticalManualMode(false);
                    }
                )
        );

        new Trigger(() -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 && (matchTime.isLessThan(20) || debugMode))
                .toggleWhenActive(
                        new InstantCommand(() -> {
                            kickstand.raise();
                            shooter.setHorizontalManualMode(true);
                            shooter.setHorizontalAngle(0);
                            shooter.disable();
                            drive.disable();
                        }),
                        new InstantCommand(() -> {
                            kickstand.drop();
                            shooter.setHorizontalManualMode(false);
                            shooter.enable();
                            drive.enable();
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(drive.goToGate());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(drive.goToBase());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> {
                    boolean success = vision.relocalize();
                    if (!success) gamepad1.rumble(300);
                });

        if (debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    .toggleWhenPressed(
                            new InstantCommand(() -> shooter.disable()),
                            new InstantCommand(() -> shooter.enable())
                    );
        }


        if (testings) {
            new InstantCommand(() -> shooter.disable());
            new InstantCommand(() -> shooter.enable());

        }

        if (!tabletopMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(this::resetPoseToNearestCorner);
        }

        new Trigger(transfer.threeArtifactsDetected(intake::isCollecting, 250))
                .whenActive(new InstantCommand(() -> gamepad1.rumble(300)));

        intake.setDefaultCommand(new RunCommand(intake::stop, intake));
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.teleOpDrive(gamepad1),
                        drive
                )
        );

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void onStart() {
        matchTime.start();
    }


    public Pose getAprilTagPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();

            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

                Pose standardFTCPose = new Pose(x, y, heading).scale(METERS_TO_INCHES);
                Pose pedroPose = FTCCoordinates.INSTANCE.convertToPedro(standardFTCPose);

                return pedroPose;
            }
        }

        return new Pose();
    }

    // kalman debugging, that includes the following:
    // pinpoint and apriltag poses v
    // covariance of the model v
    // the calculated stdevs

    public void kalmanDebug(){

        Pose2D apriltagPose2D = PoseConverter.poseToPose2D(aprilTagPipeline.getPose(), FTCCoordinates.INSTANCE);
        telemetry.addData("apriltag pose x", -apriltagPose2D.getX(DistanceUnit.INCH));
        telemetry.addData("apriltag pose y", -apriltagPose2D.getY(DistanceUnit.INCH));
        telemetry.addData("apriltag pose heading", apriltagPose2D.getHeading(AngleUnit.RADIANS) - Math.PI);

        Pose2D pinpointPose2D = PoseConverter.poseToPose2D(pinpointPose, FTCCoordinates.INSTANCE);
        telemetry.addData("pinpoint pose x", -pinpointPose2D.getX(DistanceUnit.INCH));
        telemetry.addData("pinpoint pose y", -pinpointPose2D.getY(DistanceUnit.INCH));
        telemetry.addData("pinpoint pose heading", pinpointPose2D.getHeading(AngleUnit.RADIANS) - Math.PI);

        telemetry.addData("covariance x", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getP().get(0, 0));
        telemetry.addData("covariance y", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getP().get(1, 1));
        telemetry.addData("covariance heading", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getP().get(2, 2));

        if (((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getInnovationCovariance() != null) {
            telemetry.addData("Innovation x", ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).getInnovationCovariance().get(0, 0));
            telemetry.addData("Innovation y", ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).getInnovationCovariance().get(1, 1));
            //telemetry.addData("Innovation heading", ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).getInnovationCovariance().get(2, 2));

//            telemetry.addData("cov x", ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).getcov().get(0, 0));
//            telemetry.addData("cov y", ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).getcov().get(1, 1));
//            telemetry.addData("cov heading", ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).getcov().get(2, 2));
        }


        telemetry.addData("stdev x", sizeVarianceX);
        telemetry.addData("stdev y", sizeVarianceX);
        telemetry.addData("stdev heading", sizeVarianceAngle);

        Logger.recordOutput("apriltag pose x", aprilTagPipeline.getPose().getX());
        Logger.recordOutput("apriltag pose y", aprilTagPipeline.getPose().getY());
        Logger.recordOutput("apriltag pose heading", aprilTagPipeline.getPose().getHeading());

        Logger.recordOutput("pinpoint pose x", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose().getX());
        Logger.recordOutput("pinpoint pose y", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose().getY());
        Logger.recordOutput("pinpoint pose heading", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose().getHeading());

        Logger.recordOutput("covariance x", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getP().get(0, 0));
        Logger.recordOutput("covariance y", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getP().get(1, 1));
        Logger.recordOutput("covariance heading", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getP().get(2, 2));

        Logger.recordOutput("stdev x", sizeVarianceX);
        Logger.recordOutput("stdev y", sizeVarianceY);
        Logger.recordOutput("stdev heading", sizeVarianceAngle);
    }


    // updates the kalman filter if we got a tag reading, if that's case we calculate the variance based on the the tag's size in the frame
//    public void updateKFApriltagReading() {
//        if (aprilTagPipeline.getApriltagDetection() != null) {
//            long time = System.nanoTime();
//            ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).addMeasurement(aprilTagPipeline.getPose(), time - 1_000_000L * (long)CameraUtil.getLatencyCamera());
//        }
//
//    }

    public void updateKFApriltagReading() {
        if (aprilTagPipeline.getApriltagDetection() != null) {
            // 1. Fetch the frozen hardware-level capture timestamp from the pipeline
            long rawFrameTime = aprilTagPipeline.getLatestTimestamp();

            // 2. Convert the double ms constants from KalmanConfig into nanoseconds
            long cameraLatencyNanos = (long) (KalmanConfig.CAMERA_PHYSICAL_LATENCY_MS * 1e6);
            long pinpointLatencyNanos = (long) (KalmanConfig.PINPOINT_I2C_LATENCY_MS * 1e6);

            // 3. Calculate the relative offset package
            long relativeLatencyOffset = cameraLatencyNanos - pinpointLatencyNanos;

            // 4. Align the frame time to the correct moment on the Pinpoint timeline
            long correctedTimestamp = rawFrameTime - relativeLatencyOffset;

            // 5. Inject the measured pose and the timeline-corrected timestamp into the EKF
            ((FusionLocalizer) follower.getPoseTracker().getLocalizer())
                    .addMeasurement(aprilTagPipeline.getPose(), correctedTimestamp);
        }
    }

    public long getResultTimestamp() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult == null || !llResult.isValid()) {
            return Long.MIN_VALUE;
        }

        return System.currentTimeMillis() - (long) llResult.getStaleness();
    }
    @Override
    public void run() {
        double currentTime = matchTime.getElapsed();
        double loopTimeMs = (currentTime - lastLoopTime) * 1000.0;
        lastLoopTime = currentTime;
        loopCount++;

        pinpointPose = ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose();

        updateKFApriltagReading();

        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
        robotZone.setRotation(follower.getPose().getHeading());

        shooter.setZoneCalculator(getCalculatorZone());

//        double orientationDeg = Math.toDegrees(follower.getPose().getHeading()) + 90;
//        limelight.updateRobotOrientation(orientationDeg);
//
//        // --- 1. PINPOINT UPDATE & TIMESTAMPS ---
//        pinpoint.update();
//
//        // Grab the raw boot-based nanosecond timestamp from the hardware bus
//        long pinpointArrivalNanos = pinpoint.getLatestArrivalNanos();
//
//        // Convert to Epoch Milliseconds using our calculation offset
//        long pinpointArrivalEpochMs = (pinpointArrivalNanos / 1_000_000) + nanoTimeToEpochMillisOffset;
//
//        // --- 2. LIMELIGHT UPDATE & TIMESTAMPS ---
//        long limelightCaptureEpochMs = getResultTimestamp();
//
//        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid())
//        {
//            LLResult result = limelight.getLatestResult();
//        }
//
//        long trueSensorSkewMs = 0;
//        if (limelightCaptureEpochMs != Long.MIN_VALUE) {
//            // Because they are on the exact same scale, this subtraction represents absolute latency
//            trueSensorSkewMs = pinpointArrivalEpochMs - limelightCaptureEpochMs;
//            telemetry.addData("Visual Age relative to Odo (ms)", trueSensorSkewMs);
//        } else {
//            telemetry.addData("Limelight Status", "No Target Detected");
//        }


        boolean isInsideLaunchZone = isInsideLaunchZone();
        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        shooter.setUpdateFlywheel(distanceFromLaunchZone() < 20);

        // Immediately cancel drive command if joysticks are moved
        boolean inputDetected = Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        if (!tabletopMode &&
                drive.getCurrentCommand() != null &&
                drive.getCurrentCommand() != drive.getDefaultCommand() &&
                inputDetected) {
            CommandScheduler.getInstance().cancel(drive.getCurrentCommand());
            follower.startTeleopDrive(USE_BRAKE_MODE);
        }

        // Cancel shooting if:
        // 1. We aren't in the zone
        // 2. AND we haven't explicitly overriden for this specific shot cycle
        if (shooter.getCurrentCommand() == null && overrideTimer.isDone()) {
            isOverrideActive = false;
        }

        if (!isInsideLaunchZone && !isOverrideActive) {
            CommandScheduler.getInstance().cancel(shooter.getCurrentCommand());
        }

        double goalDistance = follower.getPose().distanceFrom(
                alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL
        ) / INCHES_TO_METERS;

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        telemetry.addData("!Loop Time (ms)", "%.2f", loopTimeMs);
        telemetry.addData("!Frequency (Hz)", "%.1f", 1000.0 / loopTimeMs);

//        telemetry.addData("!Inside LAUNCH ZONE", isInsideLaunchZone);
//        telemetry.addData("!Reached angle", shooter.reachedAngle());
//        telemetry.addData("!can Shoot RPM calc ", shooter.getCanShootRPMCalc());
//        telemetry.addData("!Can shoot", shooter.getCanShoot());

        telemetry.addData("Time remaining", matchTime.getRemaining());

        kalmanDebug();

        telemetry.addData("Image size x",aprilTagPipeline.getTagSizeX());
        telemetry.addData("Image size y",aprilTagPipeline.getTagSizeY());

        telemetry.addData("Pedro Robot x", follower.getPose().getX());
        telemetry.addData("Pedro Robot y", follower.getPose().getY());
        telemetry.addData("Pedro Robot heading", follower.getPose().getHeading());
        telemetry.addData("Kalman x", -rotatedPose.getX());
        telemetry.addData("Kalman y", -rotatedPose.getY());
        telemetry.addData("Kalman heading", rotatedPose.getHeading() - Math.PI);

        double driftX = Math.abs(X_OFFSET - follower.getPose().getX());
        double driftY = Math.abs(Y_OFFSET - follower.getPose().getY());
        telemetry.addData("Drift Kalman x", driftX);
        telemetry.addData("Drift Kalman y", driftY);
        telemetry.addData("Drift Kalman total", driftX + driftY);

        double driftXPinpoint = Math.abs(X_OFFSET - pinpointPose.getX());
        double driftYPinpoint = Math.abs(Y_OFFSET - pinpointPose.getY());
        telemetry.addData("Drift pintpoint x", driftXPinpoint);
        telemetry.addData("Drift pintpoint y", driftYPinpoint);
        telemetry.addData("Drift pintpoint total", driftXPinpoint + driftYPinpoint);

        double diff = driftXPinpoint + driftYPinpoint - (driftX + driftY);
        if(diff < 0) {
            telemetry.addData("pinpoint is more accurate by", Math.abs(diff));
        }

        else if (diff > 0) {
            telemetry.addData("kalman is more accuarate by", Math.abs(diff));
        }
//        telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
//        telemetry.addData("Distance from GOAL", goalDistance);
//        telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
//        telemetry.addData("Turret target (deg)", Math.toDegrees(shooter.wrapped));
//        telemetry.addData("Turret target solution (deg)", Math.toDegrees(shooter.solution.getHorizontalAngle()));
//        telemetry.addData("Turret error (deg)", Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES));
//        telemetry.addData("Turret window (deg)", Math.toDegrees(shooter.getTurretWindow()));
//
//        telemetry.addData("hood pos", shooter.getRawHoodPosition());
//        telemetry.addData("hood angle(deg)", shooter.getHoodAngleDegrees());
//        telemetry.addData("Flywheel RPM", shooter.getRPM());
//        telemetry.addData("Filtered Flywheel RPM", shooter.filteredRPM);
//        telemetry.addData("Target solution RPM", shooter.getTargetRPM());
//        telemetry.addData("Flywheel error", Math.abs(shooter.getRPM() - shooter.getTargetRPM()));
//        telemetry.addData("Intake RPM", intake.getRPM());
//
//        telemetry.addData("Shot Hood Angle", shooter.shotHoodAngle);
//        telemetry.addData("Shot Turret Angle", shooter.shotTurretAngle);
//        telemetry.addData("Shot Flywheel RPM", shooter.shotFlywheelRPM);
//        telemetry.addData("Shot goal distance", shooter.shotGoalDistance);
//        telemetry.addData("robot vel x ", follower.getVelocity().getXComponent());
//        telemetry.addData("robot vel y ", follower.getVelocity().getYComponent());

        telemetry.update();

        Pose2d robotPose = new Pose2d(-rotatedPose.getX() / INCHES_TO_METERS, -rotatedPose.getY() / INCHES_TO_METERS, new Rotation2d(-rotatedPose.getHeading()));

//        Logger.recordOutput("Pinpoint Hardware Time (ms UTC)", "" + pinpointArrivalEpochMs);
//        Logger.recordOutput("Limelight Capture Time (ms UTC)", "" + limelightCaptureEpochMs);
//        Logger.recordOutput("skew/difference", trueSensorSkewMs);

        Logger.recordOutput("Pinpoint x", pinpoint.getPosition().getX(DistanceUnit.INCH));
        Logger.recordOutput("Pinpoint y", pinpoint.getPosition().getY(DistanceUnit.INCH));
        Logger.recordOutput("Pinpoint heading", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

        Logger.recordOutput("Diagnostics/LoopTimeMs", loopTimeMs);
        Logger.recordOutput("Diagnostics/Hz", 1000.0 / loopTimeMs);
        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Voltage", voltage);
        Logger.recordOutput("Inside LAUNCH ZONE", isInsideLaunchZone);
        Logger.recordOutput("Reached Angle", shooter.reachedAngle());
        Logger.recordOutput("Can Shoot RPM calc", shooter.getCanShootRPMCalc());
        Logger.recordOutput("Can Shoot", shooter.getCanShoot());
        Logger.recordOutput("Distance From Pose", goalDistance);
        Logger.recordOutput("Shooter/Flywheel/ Filtered RPM", shooter.filteredRPM);
        Logger.recordOutput("Shooter/Flywheel/Error", Math.abs(shooter.filteredRPM - shooter.getTargetRPM()));
        Logger.recordOutput("Shooter/Flywheel/ Target", shooter.getTargetRPM());
        Logger.recordOutput("Shooter/Hood/ Raw Position", shooter.getRawHoodPosition());
        Logger.recordOutput("Shooter/Hood/ Angle (deg)", shooter.getHoodAngleDegrees());
        Logger.recordOutput("Turret/Turret/ Angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        Logger.recordOutput("Turret/Turret/ Angle Target (deg)", Math.toDegrees(shooter.wrapped));
        Logger.recordOutput("Turret/Turret/ Angle Error (deg)", Math.abs(Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES)));
        Logger.recordOutput("Turret/Turret/ Turret window (deg)", Math.toDegrees(shooter.getTurretWindow()));
    }

    @Override
    public void end() {
        Settings.set("pose", follower.getPose(), false);
    }

    private boolean isShootingAllowed() {
        return isInsideLaunchZonePredictive() || isOverrideActive;
    }

    public boolean isInsideLaunchZone() {
        boolean insideClose = robotZone.isInside(closeLaunchZone);
        boolean insideFar = robotZone.isInside(farLaunchZone);
        return insideClose || insideFar;
    }

    public boolean isInsideLaunchZonePredictive() {
        // Project future X and Y based on current velocity
        final double PREDICTION_TIME = 0.3;
        double futureX = follower.getPose().getX() + (follower.getVelocity().getXComponent() * PREDICTION_TIME);
        double futureY = follower.getPose().getY() + (follower.getVelocity().getYComponent() * PREDICTION_TIME);

        // Create a temporary zone for the future position
        futureRobotZone.setPosition(futureX, futureY);
        futureRobotZone.setRotation(follower.getPose().getHeading());

        return futureRobotZone.isInside(closeLaunchZone) || futureRobotZone.isInside(farLaunchZone);
    }

    public double distanceFromLaunchZone() {
        return Math.min(robotZone.distanceTo(closeLaunchZone), robotZone.distanceTo(farLaunchZone));
    }

    private void resetPoseToNearestCorner() {
        Pose newPose;
        if (alliance == Alliance.RED) {
            newPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
        } else {
            newPose = new Pose(141.5 - X_OFFSET, Y_OFFSET, Math.toRadians(180));
        }

        follower.setPose(new Pose(newPose.getX(), newPose.getY(), newPose.getHeading()));
        follower.startTeleopDrive(USE_BRAKE_MODE);
    }

    private LaunchZone getCalculatorZone() {
        return robotZone.distanceTo(closeLaunchZone) < robotZone.distanceTo(farLaunchZone) ? LaunchZone.CLOSE : LaunchZone.FAR;
    }
}
