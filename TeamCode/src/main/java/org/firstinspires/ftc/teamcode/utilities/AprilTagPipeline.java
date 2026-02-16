package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.cameraMatrix;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.distCoeffs;

import android.graphics.Canvas;

import com.pedropathing.geometry.Pose;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.teamcode.config.BlackWhiteCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

public class AprilTagPipeline extends TimestampedOpenCvPipeline
{
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    Mat undistored = new Mat();
    MatOfDouble dist = new MatOfDouble(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);
    Mat matrix = new Mat(3, 3, CvType.CV_64F);
    public AprilTagPipeline()
    {
        this.processor = new AprilTagProcessor.Builder()

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
        processor.setDecimation(5);

        matrix.put(0, 0,
                cameraMatrix[0], cameraMatrix[1], cameraMatrix[2],
                cameraMatrix[3], cameraMatrix[4], cameraMatrix[5],
                cameraMatrix[6], cameraMatrix[7], cameraMatrix[8]);
    }

    public void noteCalibrationIdentity(CameraCalibrationIdentity ident)
    {
        this.ident = ident;
    }

    @Override
    public void init(Mat firstFrame)
    {
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, firstFrame.width(), firstFrame.height());
        processor.init(firstFrame.width(), firstFrame.height(), calibration);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Calib3d.undistort(input, undistored, matrix, dist);
        Object drawCtx = processor.processFrame(undistored, captureTimeNanos);
        requestViewportDrawHook(drawCtx);
        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    // gives the current relative apriltag position
    public Pose getRobotPose(double turretAngle) {

        if (!processor.getDetections().isEmpty()) {

            AprilTagDetection detection = processor.getDetections().get(0);

            double relX = detection.ftcPose.x;
            double relY = detection.ftcPose.y;
            double relHeading = Math.toRadians(detection.ftcPose.bearing);

            double cosT = Math.cos(turretAngle);
            double sinT = Math.sin(turretAngle);

            // Rotate measurement into robot frame
            double rotatedX = relX * cosT - relY * sinT;
            double rotatedY = relX * sinT + relY * cosT;

            // Camera offset from robot center (inches)
            double camOffsetX = BlackWhiteCamera.offsetX_turret;
            double camOffsetY = BlackWhiteCamera.offsetY_turret;

            // Translate to robot center
            double robotX = rotatedX - camOffsetX;
            double robotY = rotatedY - camOffsetY;

            double robotHeading = relHeading + turretAngle;

            return new Pose(robotX, robotY, robotHeading);
        }

        return new Pose(0,0,0);
    }


    public AprilTagDetection getApriltagDetection() {
        if (!processor.getDetections().isEmpty())
            return processor.getDetections().get(0);
        return null;
    }

}
