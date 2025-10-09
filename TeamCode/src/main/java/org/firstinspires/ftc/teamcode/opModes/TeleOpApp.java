package org.firstinspires.ftc.teamcode.opModes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

import java.util.List;

@Config
@TeleOp
public class TeleOpApp extends ComplexOpMode {
    private Follower follower;
    private Shooter shooter;
    private Limelight3A limelight;

    Motor intake;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    SimpleServo cameraServo;
    AprilTagProcessor aprilTag;

    VisionPortal visionPortal;
    double bearingAngle;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            -7.5, 1.5, 14.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public OpenCvWebcam webcam;
    AprilTagPipeline aprilTagPipeline;

    public static double Ki = 0.00008;
    public static double Kp = 0.200011;
    public static double Kd = 0.002;
    ElapsedTime timer = new ElapsedTime();
     double lastError = 0;
     double integralSum = 0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();*/

        //shooter = new Shooter(hardwareMap, follower.poseTracker);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        /*new Trigger(() -> gamepad1.right_trigger > 0.1)
                .whenActive(new ShootCommand(shooter));
*/

        intake = new Motor(hardwareMap, "intake");
        schedule(
                // TODO: Set shooter angle to GOAL
        );

        cameraServo = new SimpleServo(
                hardwareMap, "cameraServo", 0, 300, AngleUnit.DEGREES
        );
        //cameraServo.setPosition(0);
        //initAprilTag();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(688.9972, 688.5173, 613.914, 397.1161)
                // ... these parameters are fx, fy, cx, cy.

                .build();


        initAprilTag();
        /*aprilTagPipeline = new AprilTagPipeline(aprilTag);

        webcam.setPipeline(aprilTagPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Webcam", "Error: " + errorCode);
            }
        });*/

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(688.9972, 688.5173, 613.914, 397.1161)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.n
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                bearingAngle = detection.ftcPose.bearing;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()




    public double correctAngle(double bearingAngle) {
        double error = 23 - bearingAngle;
        double position = bearingAngle;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        lastError = error;
        timer.reset();
        return out;

    }
    @Override
    public void run() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        gamepadEx1.readButtons();
        if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
        {
            intake.set(-0.75);
        }
        if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            intake.set(-1);
        }
        else {
            intake.set(0);
        }

        /*LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.METER);
            YawPitchRollAngles orientation = result.getBotpose().getOrientation();

            Logger.recordOutput("Limelight Pose", new Pose2d(botpose.x, botpose.y, new Rotation2d(orientation.getYaw(AngleUnit.RADIANS))));
            telemetry.addData("LL x", botpose.x);
            telemetry.addData("LL y", botpose.y);
            telemetry.addData("LL z", botpose.z);
            telemetry.addData("LL heading", orientation.getYaw(AngleUnit.RADIANS));
        }*/

        telemetry.addData("Robot x", follower.getPose().getX());
        telemetry.addData("Robot y", follower.getPose().getY());
        telemetry.addData("Robot heading", follower.getPose().getHeading());

        double inchesToMeters = 39.37;
        Pose2d robotPose = new Pose2d(follower.getPose().getX() / inchesToMeters, follower.getPose().getY() / inchesToMeters, new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("Robot Pose", robotPose);

        //telemetryAprilTag();

        /*if (aprilTagPipeline.getApriltagDetection()!= null)
            bearingAngle = aprilTagPipeline.getApriltagDetection().ftcPose.bearing;*/

        telemetryAprilTag();

        /*if (gamepad1.circleWasPressed()) {
            cameraServo.rotateByAngle(23 - bearingAngle);
        }*/

        cameraServo.rotateByAngle(correctAngle(bearingAngle));

        if (gamepad1.squareWasPressed()) {
            cameraServo.rotateByAngle(5);
        }

        if (gamepad1.triangleWasPressed()) {
            cameraServo.rotateByAngle(-5);
        }

        telemetry.addData("rotate angle: ", -bearingAngle);
        telemetry.addData("current angle: ", cameraServo.getAngle());
        telemetry.update();
    }
}
