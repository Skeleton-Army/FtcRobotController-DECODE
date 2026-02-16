package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.cameraMatrix;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.distCoeffs;

import android.graphics.Canvas;

import com.pedropathing.geometry.Pose;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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
    public AprilTagPipeline(AprilTagProcessor processor)
    {
        this.processor = processor;

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
    public Pose getRelativePose() {
        if (!processor.getDetections().isEmpty())  {
            AprilTagDetection detection = processor.getDetections().get(0);

            OpModeManager.getTelemetry().addLine(String.format("XYZ raw %6.1f %6.1f %6.1f  (inch)",
                    detection.rawPose.x,
                    detection.rawPose.y,
                    detection.rawPose.z));

            OpModeManager.getTelemetry().addLine(String.format("XYZ ftcpose %6.1f %6.1f %6.1f  (inch)",
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    detection.ftcPose.z));

            OpModeManager.getTelemetry().addLine(String.format("XYZ ftcpose %6.1f %6.1f %6.1f  (inch)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));




            return new Pose(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.bearing);
            /*Position detectionPos = detection.robotPose.getPosition();

            follower.setPose(new Pose(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90)));
            return new Pose(detectionPos.x, detectionPos.y, Math.toRadians(detection.robotPose.getOrientation().getYaw() + 90));*/
        }
        return new Pose(0,0,0);
    }

    public AprilTagDetection getApriltagDetection() {
        if (!processor.getDetections().isEmpty())
            return processor.getDetections().get(0);
        return null;
    }

}
