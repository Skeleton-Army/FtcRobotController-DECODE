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
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.settings.Settings;

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
import org.firstinspires.ftc.teamcode.utilities.FollowerManager;
import org.firstinspires.ftc.teamcode.utilities.FusionLocalizer;
import org.firstinspires.ftc.teamcode.utilities.WelfordVariance;
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

    private double totalTraveledX = 0;
    private double totalTraveledY = 0;

    private boolean autoFireEnabled = true;

    private long lastTimeNanos = 0;

    private long highSpeedStartNanos = 0;
    private boolean wasMovingFast = false;

    double calculatedLatencySeconds = 0;
    double calculatedLatencyMillis = 0;
    double positionGap = 0;

    Pose pinpointPose;

    WelfordVariance welfordVarianceX =  new WelfordVariance();
    WelfordVariance welfordVarianceY =  new WelfordVariance();
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

        follower = FollowerManager.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setMaxPower(1);

        Pose startPose = new Pose(X_OFFSET, Y_OFFSET, Math.toRadians(0));
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

        autoFireEnabled = Settings.get("auto_fire", true);

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

        // Fire immediately when entering zone
        new Trigger(() -> autoFireEnabled
                && drive.isInsideLaunchZonePredictive()
                && shooter.getCanShoot()
                && (shooter.getCurrentCommand() == null || shooter.getCurrentCommand() == shooter.getDefaultCommand())
                && (transfer.getCurrentCommand() == null || transfer.getCurrentCommand() == transfer.getDefaultCommand())
                && (intake.getCurrentCommand() == null || intake.getCurrentCommand() == intake.getDefaultCommand())
        )
                .whenActive(new ShootCommand(shooter, intake, transfer, drive));

        // Kick automatically when exiting the launch zone
        new Trigger(drive::isInsideLaunchZonePredictive)
                .whenInactive(transfer.kick());

        // Toggle auto-fire on-off
        gamepadEx1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(
                        new InstantCommand(() -> {
                            autoFireEnabled = !autoFireEnabled;
                            gamepad1.rumble(200);
                        })
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setVerticalOffset(shooter.getVerticalOffset() + 0.05))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setVerticalOffset(shooter.getVerticalOffset() - 0.05))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setHorizontalOffset(shooter.getHorizontalOffset() + 0.003))
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .or(gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT))
                .whileActiveContinuous(
                        new InstantCommand(() -> shooter.setHorizontalOffset(shooter.getHorizontalOffset() - 0.003))
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

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(drive.goToBase());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.SHARE)
                .whenPressed(drive.LoadingZoneCycle());


//        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(() -> {
//                    boolean success = vision.relocalize();
//                    if (!success) gamepad1.rumble(300);
//                });

        if (debugMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    .toggleWhenPressed(
                            new InstantCommand(() -> shooter.disable()),
                            new InstantCommand(() -> shooter.enable())
                    );
        }

        if (!tabletopMode) {
            gamepadEx1.getGamepadButton(GamepadKeys.Button.PS)
                    .whenPressed(this::dynamicPoseReset);
        }

        new Trigger(transfer.threeArtifactsDetected(intake::isCollecting, 250))
                .whenActive(new InstantCommand(() -> gamepad1.rumble(300)));

        // Cancel shooting when turret wraps around
        new Trigger(shooter::getJustWrapped)
                .whenActive(new InstantCommand(() -> CommandScheduler.getInstance().cancel(shooter.getCurrentCommand())));

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


    public void calculateCameraLatencyEmpirically() {
        Pose2D apriltagPose2D = PoseConverter.poseToPose2D(aprilTagPipeline.getPose(), FTCCoordinates.INSTANCE);
        Pose2D pinpointPose2D = PoseConverter.poseToPose2D(((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose(), FTCCoordinates.INSTANCE);

        Pose pinpoint = ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getVelocity();

        double velX = pinpoint.getX();
        double velY = pinpoint.getY();
        double netVelocity = Math.hypot(velX, velY);

        // Filter: Is the robot traveling fast? (e.g., > 18 inches/second)
        if (netVelocity > 18.0) {
            if (!wasMovingFast) {
                // The robot just crossed the speed threshold; start the timer
                highSpeedStartNanos = System.nanoTime();
                wasMovingFast = true;
            }

            // Calculate how long we have maintained this high speed
            double timeSpentFastSeconds = (System.nanoTime() - highSpeedStartNanos) / 1e9;

            // ONLY log data after maintaining high speed for 0.4 seconds (ensures kickoff is finished)
            if (timeSpentFastSeconds > 0.4 && aprilTagPipeline.getApriltagDetection() != null) {

                double deltaX = (-pinpointPose2D.getX(DistanceUnit.INCH)) - (-apriltagPose2D.getX(DistanceUnit.INCH));
                double deltaY = (-pinpointPose2D.getY(DistanceUnit.INCH)) - (-apriltagPose2D.getY(DistanceUnit.INCH));
                positionGap = Math.hypot(deltaX, deltaY);

                calculatedLatencySeconds = positionGap / netVelocity;
                calculatedLatencyMillis = calculatedLatencySeconds * 1000.0;

                telemetry.addData("[TUNER] Status", "STEADY CRUISE - Calculating");
                telemetry.addData("[TUNER] Position Gap (in)", positionGap);
                telemetry.addData("[TUNER] CALC LATENCY (ms)", calculatedLatencyMillis);
            } else {
                telemetry.addData("[TUNER] Status", "Waiting for speed to stabilize...");
            }
        } else {
            // Robot slowed down or stopped; reset the window
            wasMovingFast = false;
            telemetry.addData("[TUNER] Status", "Too Slow / Stopped");
        }
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

        Pose2D apriltagPose2D = poseToPose2D(aprilTagPipeline.getPose(), FTCCoordinates.INSTANCE);
        telemetry.addData("apriltag pose x", -apriltagPose2D.getX(DistanceUnit.INCH));
        telemetry.addData("apriltag pose y", -apriltagPose2D.getY(DistanceUnit.INCH));
        telemetry.addData("apriltag pose heading", apriltagPose2D.getHeading(AngleUnit.RADIANS) - Math.PI);

        pinpointPose = ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose();
        Pose2D pinpointPos2D = poseToPose2D(pinpointPose, FTCCoordinates.INSTANCE);
        telemetry.addData("pinpoint pose x", -pinpointPos2D.getX(DistanceUnit.INCH));
        telemetry.addData("pinpoint pose y", -pinpointPos2D.getY(DistanceUnit.INCH));
        telemetry.addData("pinpoint pose heading", pinpointPos2D.getHeading(AngleUnit.RADIANS) - Math.PI);

        telemetry.addData("covariance x", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getCovariance().get(0, 0));
        telemetry.addData("covariance y", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getCovariance().get(1, 1));
        telemetry.addData("covariance heading", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getCovariance().get(2, 2));

        telemetry.addData("[TUNER] Position Gap (in)", positionGap);
        telemetry.addData("[TUNER] CALC LATENCY (ms)", calculatedLatencyMillis);

        telemetry.addData("stdev x", sizeVarianceX);
        telemetry.addData("stdev y", sizeVarianceX);
        telemetry.addData("stdev heading", sizeVarianceAngle);

        Logger.recordOutput("apriltag pose x", aprilTagPipeline.getPose().getX());
        Logger.recordOutput("apriltag pose y", aprilTagPipeline.getPose().getY());
        Logger.recordOutput("apriltag pose heading", aprilTagPipeline.getPose().getHeading());

        Logger.recordOutput("covariance x", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getCovariance().get(0, 0));
        Logger.recordOutput("covariance y", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getCovariance().get(1, 1));
        Logger.recordOutput("covariance heading", ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getCovariance().get(2, 2));

        Logger.recordOutput("stdev x", sizeVarianceX);
        Logger.recordOutput("stdev y", sizeVarianceY);
        Logger.recordOutput("stdev heading", sizeVarianceAngle);
    }


//     updates the kalman filter if we got a tag reading, if that's case we calculate    the variance based on the the tag's size in the frame
//    public void updateKFApriltagReading() {
//        if (aprilTagPipeline.getApriltagDetection() != null
//                && Math.abs(aprilTagPipeline.getPose().getHeading() - ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).getDeadReckoning().getPose().getHeading()) < 10
//                && follower.getVelocity().getMagnitude() < 0.5
//                && follower.getAngularVelocity() < 0.5
//        ) {
//            long time = System.nanoTime();
//            ((FusionLocalizer)follower.getPoseTracker().getLocalizer()).addMeasurement(aprilTagPipeline.getPose(), time - 1_000_000L * (long)CameraUtil.getLatencyCamera());
//        }
//
//    }

    public void updateKFApriltagReading() {
        if (aprilTagPipeline.getApriltagDetection() != null
                && KalmanConfig.enableMeasurements
                && drive.distanceFromLaunchZone() < 20
                && Math.abs(follower.getAngularVelocity()) < 0.5) {
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

        if (loopCount > 1) {
            double deltaX = Math.abs(follower.poseTracker.getRawPose().getX() - follower.poseTracker.getPreviousPose().getX());
            double deltaY = Math.abs(follower.poseTracker.getRawPose().getY() - follower.poseTracker.getPreviousPose().getY());

            if (deltaX > 0.05) totalTraveledX += deltaX;
            if (deltaY > 0.05) totalTraveledY += deltaY;
        }

        updateKFApriltagReading();

        double voltage = voltageSensor.getVoltage();

        shooter.updateVoltage(voltage);
        shooter.setUpdateFlywheel(drive.distanceFromLaunchZone() < 40);

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

        if (!drive.isInsideLaunchZonePredictive() && !isOverrideActive) {
            Command currentShooterCommand = shooter.getCurrentCommand();

            // Cancel shooting ONLY if we are running a standalone ShootCommand, not a complex macro like CloseCycleCommand
            if (currentShooterCommand instanceof ShootCommand) {
                CommandScheduler.getInstance().cancel(currentShooterCommand);
            }
        }

        double goalDistance = follower.getPose().distanceFrom(
                alliance == Alliance.RED ? GoalPositions.RED_GOAL : GoalPositions.BLUE_GOAL
        ) / INCHES_TO_METERS;

        Pose rotatedPose = follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        telemetry.addData("!Loop Time (ms)", "%.2f", loopTimeMs);
        telemetry.addData("!Frequency (Hz)", "%.1f", 1000.0 / loopTimeMs);

        telemetry.addData("!Inside LAUNCH ZONE", drive.isInsideLaunchZonePredictive());
        telemetry.addData("!Reached angle", shooter.reachedAngle());
        telemetry.addData("!can Shoot RPM calc ", shooter.getCanShootRPMCalc());
        telemetry.addData("!Can shoot", shooter.getCanShoot());
        telemetry.addData("!Is currently shooting", shooter.getCurrentCommand() != null);

        telemetry.addData("Time remaining", matchTime.getRemaining());

        kalmanDebug();

        //telemetry.addData(aprilTagPipeline.getProcessor().getDetections().get(0).rawPose);

        telemetry.addData("x apriltag - x pinpoint", aprilTagPipeline.getPose().minus(pinpointPose).getX());
        welfordVarianceX.update(aprilTagPipeline.getPose().minus(pinpointPose).getX());
        telemetry.addData("mean x error", welfordVarianceX.mean());
        telemetry.addData("y apriltag - y pinpoint", aprilTagPipeline.getPose().minus(pinpointPose).getY());
        welfordVarianceY.update(aprilTagPipeline.getPose().minus(pinpointPose).getY());
        telemetry.addData("mean y error", welfordVarianceY.mean());
        telemetry.addData("heading apriltag - heading pinpoint", aprilTagPipeline.getPose().minus(pinpointPose).getHeading());

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
        double driftTotal = driftX + driftY;
        telemetry.addData("Drift x", driftX);
        telemetry.addData("Drift y", driftY);
        telemetry.addData("Drift total", driftTotal);

        telemetry.addData("Total Traveled X", totalTraveledX);
        telemetry.addData("Total Traveled Y", totalTraveledY);
        telemetry.addData("Total Traveled XY", totalTraveledX + totalTraveledY);

        telemetry.addData("Pedro Robot x", follower.getPose().getX());
        telemetry.addData("Pedro Robot y", follower.getPose().getY());
        telemetry.addData("Pedro Robot heading", follower.getPose().getHeading());
        telemetry.addData("Robot x", -rotatedPose.getX());
        telemetry.addData("Robot y", -rotatedPose.getY());
        telemetry.addData("Robot heading", rotatedPose.getHeading() - Math.PI);
        telemetry.addData("Robot velocity", follower.poseTracker.getVelocity());
        telemetry.addData("Distance from GOAL", goalDistance);
        telemetry.addData("Turret angle (deg)", shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret target (deg)", Math.toDegrees(shooter.wrapped));
        telemetry.addData("Turret target solution (deg)", Math.toDegrees(shooter.solution.getHorizontalAngle()));
        telemetry.addData("Turret error (deg)", Math.toDegrees(shooter.wrapped) - shooter.getTurretAngle(AngleUnit.DEGREES));
        telemetry.addData("Turret window (deg)", Math.toDegrees(shooter.getTurretWindow()));
        telemetry.addData("Turret horizontal offset (deg)", Math.toDegrees(shooter.getHorizontalOffset()));

        double driftXPinpoint = Math.abs(X_OFFSET - pinpointPose.getPose().getX());
        double driftYPinpoint = Math.abs(Y_OFFSET - pinpointPose.getPose().getY());
        double driftTotalPinpoint = driftXPinpoint + driftYPinpoint;
        telemetry.addData("Drift x", driftXPinpoint);
        telemetry.addData("Drift y", driftYPinpoint);
        telemetry.addData("Drift total pinpoint", driftTotalPinpoint);

        if (driftTotalPinpoint > driftTotal)
            telemetry.addData("Kalman more accurate by", driftTotalPinpoint - driftTotal);

        else if (driftTotalPinpoint <= driftTotal)
            telemetry.addData("pinpoint is more accurate by", driftTotal- driftTotalPinpoint);

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

        Logger.recordOutput("Diagnostics/LoopTimeMs", loopTimeMs);
        Logger.recordOutput("Diagnostics/Hz", 1000.0 / loopTimeMs);
        Logger.recordOutput("Robot Pose", robotPose);
        Logger.recordOutput("Voltage", voltage);
        Logger.recordOutput("Inside LAUNCH ZONE", drive.isInsideLaunchZonePredictive());
        Logger.recordOutput("Reached Angle", shooter.reachedAngle());
        Logger.recordOutput("Can Shoot RPM calc", shooter.getCanShootRPMCalc());
        Logger.recordOutput("Can Shoot", shooter.getCanShoot());
        Logger.recordOutput("Is Currently Shooting", shooter.getCurrentCommand() != null);
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
        Logger.recordOutput("Turret/Turret/ Turret offset (deg)", Math.toDegrees(shooter.getHorizontalOffset()));
    }

    private boolean isShootingAllowed() {
        return drive.isInsideLaunchZonePredictive() || isOverrideActive;
    }

    private void dynamicPoseReset() {
        final double VELOCITY_THRESHOLD = 1.0;
        if (follower.getVelocity().getMagnitude() > VELOCITY_THRESHOLD) {
            return;
        }

        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double heading = currentPose.getHeading();

        final double WALL_THRESHOLD = 20.0;
        final double FIELD_LIMIT = 141.5;

        boolean nearLeft = currentX < WALL_THRESHOLD;
        boolean nearRight = currentX > (FIELD_LIMIT - WALL_THRESHOLD);
        boolean nearBottom = currentY < WALL_THRESHOLD;
        boolean nearTop = currentY > (FIELD_LIMIT - WALL_THRESHOLD);

        if (!nearLeft && !nearRight && !nearBottom && !nearTop) return;

        double targetX = currentX;
        double targetY = currentY;
        double targetHeading = heading;

        // Normalize heading to [0, 2PI)
        double normalizedHeading = MathFunctions.normalizeAngle(heading);
        boolean facingRight = normalizedHeading < Math.toRadians(45) || normalizedHeading >= Math.toRadians(315);
        boolean facingUp    = normalizedHeading >= Math.toRadians(45) && normalizedHeading < Math.toRadians(135);
        boolean facingLeft  = normalizedHeading >= Math.toRadians(135) && normalizedHeading < Math.toRadians(225);
        boolean facingDown  = normalizedHeading >= Math.toRadians(225) && normalizedHeading < Math.toRadians(315);

        boolean frontFacingHorizontal = facingRight || facingLeft;
        boolean frontFacingVertical   = facingUp || facingDown;

        // --- X Axis Snap Logic ---
        if (nearLeft) {
            targetX = getDistanceToCenter(frontFacingHorizontal, facingLeft);
        } else if (nearRight) {
            targetX = FIELD_LIMIT - getDistanceToCenter(frontFacingHorizontal, facingRight);
        }

        // --- Y Axis Snap Logic ---
        if (nearBottom) {
            targetY = getDistanceToCenter(frontFacingVertical, facingDown);
        } else if (nearTop) {
            targetY = FIELD_LIMIT - getDistanceToCenter(frontFacingVertical, facingUp);
        }

        // --- Corner Heading Reset Logic ---
        // Reset heading if the robot is in a corner
        boolean inCorner = (nearLeft || nearRight) && (nearBottom || nearTop);
        if (inCorner) {
            if (facingRight) {
                targetHeading = Math.toRadians(0);
            } else if (facingUp) {
                targetHeading = Math.toRadians(90);
            } else if (facingLeft) {
                targetHeading = Math.toRadians(180);
            } else if (facingDown) {
                targetHeading = Math.toRadians(270);
            }
        }

        follower.setPose(new Pose(targetX, targetY, targetHeading));
        follower.startTeleopDrive(USE_BRAKE_MODE);
        gamepad1.rumble(300);
    }

    /**
     * Calculates the distance from the wall to the tracking center point of the robot.
     */
    private double getDistanceToCenter(boolean axisPerpendicularToFront, boolean frontIsFacingWall) {
        final double INTAKE_EXTENSION = 3.0; // How much the intake extends from the front

        if (axisPerpendicularToFront) {
            return frontIsFacingWall ? (X_OFFSET + INTAKE_EXTENSION) : X_OFFSET;
        }
        return Y_OFFSET;
    }
}
