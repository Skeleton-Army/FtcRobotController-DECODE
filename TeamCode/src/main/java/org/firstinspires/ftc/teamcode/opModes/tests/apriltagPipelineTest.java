package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.BlackWhiteCamera;
import org.firstinspires.ftc.teamcode.utilities.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Black&WhiteApriltag", group = "Tests")
public class apriltagPipelineTest extends ComplexOpMode {

    AprilTagProcessor aprilTag;

    AprilTagPipeline aprilTagPipeline;

    @Override
    public void initialize() {
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + BlackWhiteCamera.pitchAngle,0,0))
                .setLensIntrinsics(
                        BlackWhiteCamera.cameraMatrix[0],
                        BlackWhiteCamera.cameraMatrix[4],
                        BlackWhiteCamera.cameraMatrix[2],
                        BlackWhiteCamera.cameraMatrix[5]
                )

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(1413.91, 1413.91, 965.446, 529.378)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        aprilTag.setDecimation(5);

        aprilTagPipeline = new AprilTagPipeline(aprilTag);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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
        });
    }
    @Override
    public void run() {

        telemetry.addData("ftcPose",aprilTagPipeline.getRelativePose());
        telemetry.update();
    }
}
