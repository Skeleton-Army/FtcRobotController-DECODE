package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.cameraMatrix;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.distCoeffs;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.offsetX_turret;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.offsetY_turret;

import android.graphics.Canvas;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

import java.util.List;

public class AprilTagPipeline extends TimestampedOpenCvPipeline
{
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    Mat undistorted = new Mat();
    MatOfDouble dist = new MatOfDouble(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);
    Mat matrix = new Mat(3, 3, CvType.CV_64F);

    private final double baseFx = BlackWhiteCamera.cameraMatrix[0];
    private final double baseFy = BlackWhiteCamera.cameraMatrix[4];
    private final double baseCx = BlackWhiteCamera.cameraMatrix[2];
    private final double baseCy = BlackWhiteCamera.cameraMatrix[5];

    public AprilTagPipeline()
    {
        this.processor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + BlackWhiteCamera.pitchAngle, 0, 0))
                .setLensIntrinsics(baseFx, baseFy, baseCx, baseCy)
                .build();

        // High decimation downscales processing internal structures to keep FPS high
        // while preserving full 1280x720 3D triangulation coordinates.
        processor.setDecimation(3);

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
        Calib3d.undistort(firstFrame, undistorted, matrix, dist);
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, undistorted.width(), undistorted.height());
        processor.init(undistorted.width(), undistorted.height(), calibration);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        // Undistort full image array smoothly
        Calib3d.undistort(input, undistorted, matrix, dist);

        // Run standard detection sweep
        Object drawCtx = processor.processFrame(undistorted, captureTimeNanos);
        requestViewportDrawHook(drawCtx);

        return undistorted;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    public List<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }

    public Pose getRobotPose(double turretAngle) {
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            double relX = detection.ftcPose.x;
            double relY = detection.ftcPose.y;
            double relHeading = Math.toRadians(detection.ftcPose.bearing);

            double cosT = Math.cos(turretAngle);
            double sinT = Math.sin(turretAngle);

            double rotatedX = relX * cosT - relY * sinT;
            double rotatedY = relX * sinT + relY * cosT;

            return new Pose(rotatedX - offsetX_turret, rotatedY - offsetY_turret, relHeading + turretAngle);
        }
        return new Pose(0, 0, 0);
    }

    public Pose getPedroPose(double robotHeading, double turretHeading) {
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) return new Pose(0, 0, 0);

        AprilTagDetection detection = detections.get(0);
        if (detection.metadata == null) return new Pose(0, 0, 0);

        Pose2D tagfieldPos2D = (detection.id == 20)
                ? new Pose2D(DistanceUnit.METER, -1.482, -1.413, AngleUnit.RADIANS, 0)
                : new Pose2D(DistanceUnit.METER, -1.482, 1.413, AngleUnit.RADIANS, 0);

        Pose tagfieldPose = PoseConverter.pose2DToPose(tagfieldPos2D, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        double tagFieldX = tagfieldPose.getX();
        double tagFieldY = tagfieldPose.getY();

        double thetaCam = robotHeading + turretHeading;
        double fieldDistX = (detection.ftcPose.y * Math.cos(thetaCam)) - (-detection.ftcPose.x * Math.sin(thetaCam));
        double fieldDistY = (detection.ftcPose.y * Math.sin(thetaCam)) + (-detection.ftcPose.x * Math.cos(thetaCam));

        double offsetFieldX = (offsetY_turret * Math.cos(robotHeading)) - (offsetX_turret * Math.sin(robotHeading));
        double offsetFieldY = (offsetY_turret * Math.sin(robotHeading)) + (offsetX_turret * Math.cos(robotHeading));

        return new Pose(tagFieldX - fieldDistX - offsetFieldX, tagFieldY - fieldDistY - offsetFieldY, robotHeading);
    }

    public AprilTagDetection getApriltagDetection() {
        List<AprilTagDetection> detections = processor.getDetections();
        return (detections != null && !detections.isEmpty()) ? detections.get(0) : null;
    }

    public AprilTagProcessor getProcessor() {
        return processor;
    }
}