package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.USE_BRAKE_MODE;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utilities.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.utilities.CameraUtil;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.firstinspires.ftc.teamcode.utilities.WelfordVariance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Measurement STDEV Tuner", group = "Tuners")
@Config("Measurement STDEV Tuner")
public class MeasurementStdevTunerTest extends ComplexOpMode {

    private static final Position cameraPosition = new Position(
            DistanceUnit.INCH,
            0,
            0,
            0,
            0
    );
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -70, //-70
            0,
            0
    );
    public static boolean on = false;
    public static double actualX = 72;
    public static double actualY = 72;
    public static double actualHeading = Math.toRadians(45);
    private final WelfordVariance varianceX = new WelfordVariance();
    private final WelfordVariance varianceY = new WelfordVariance();
    private final WelfordVariance varianceHeading = new WelfordVariance();
    private AprilTagProcessor processor;

    AprilTagPipeline aprilTagPipeline;

    private Follower follower;

    private Drive drive;
    @Override
    public void initialize() {

        aprilTagPipeline = new AprilTagPipeline();
        CameraUtil.configureWebcam(aprilTagPipeline, hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(USE_BRAKE_MODE);
        follower.setMaxPower(1);

        follower.setPose(new Pose());

        drive = new Drive(follower, Alliance.BLUE);

        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.teleOpDrive(gamepad1),
                        drive
                )
        );
        /*rocessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(700.34, 697.2591, 615.85, 359.44)
                .setDrawCubeProjection(true)
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();*/

    }

    @Override
    public void run() {

            if (on) {
                List<AprilTagDetection> detections = aprilTagPipeline.getProcessor().getDetections();
                telemetry.addData("AprilTag/Detections", detections.size());

                for (AprilTagDetection detection : detections) {
                    if (detection.metadata == null || detection.metadata.name.contains("Obelisk")) continue;

                    telemetry.addData(detection.metadata.name, detection.robotPose);

                    Pose pose = new Pose(
                            detection.robotPose.getPosition().y + 72,
                            -detection.robotPose.getPosition().x + 72,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                    );

                    telemetry.addData("Apriltag x", pose.getX());
                    telemetry.addData("Apriltag y", pose.getY());
                    telemetry.addData("Apriltag angle", pose.getHeading());

                    varianceX.update(actualX - pose.getX());
                    varianceY.update(actualY - pose.getY());
                    varianceHeading.update(actualHeading - pose.getHeading());
                }
            }

            telemetry.addData("variance x", varianceX.variance());
            telemetry.addData("variance y", varianceY.variance());
            telemetry.addData("variance heading", varianceHeading.variance());

            telemetry.addData("stdev x", varianceX.stdDev());
            telemetry.addData("stdev y", varianceY.stdDev());
            telemetry.addData("stdev heading", varianceHeading.stdDev());

            telemetry.addData("mean x", varianceX.mean());
            telemetry.addData("mean y", varianceY.mean());
            telemetry.addData("mean heading", varianceHeading.mean());

            telemetry.addData("tag size x", aprilTagPipeline.getTagSizeX());
            telemetry.addData("tag size Y", aprilTagPipeline.getTagSizeY());
            telemetry.addData("tag Area", aprilTagPipeline.getTagSizeArea());
            CameraUtil.printStats();

            follower.update();
            telemetry.update();
    }

}