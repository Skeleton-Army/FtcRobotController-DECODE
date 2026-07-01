package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.cameraMatrix;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.distCoeffs;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.offsetX_turret;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.offsetY_turret;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.relativePos;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.skeletonarmy.marrow.OpModeManager;

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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.List;

@Config
public class AprilTagPipeline extends TimestampedOpenCvPipeline
{
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    private Mat fullBlackCanvas = new Mat();
    private Mat undistortedCrop = new Mat();
    private MatOfDouble dist = new MatOfDouble(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);
    private Mat matrix = new Mat(3, 3, CvType.CV_64F);
    private Mat roiMatrix = new Mat(3, 3, CvType.CV_64F);

    // --- High-Precision Timing Anchor ---
    private volatile long latestCaptureTimeNanos = 0;

    // --- Permanent Static Crop ---
    private final double BOTTOM_CROP_PERCENT = 0.20; // Cuts off the bottom 20% of the image entirely

    // --- Dynamic Adaptive ROI Bounds ---
    private Rect roiRect = null;
    public static double TAG_PADDING_PERCENT = 0.5;
    public static double TAG_PADDING_PERCENT_WIDTH = 1;
    private boolean useDynamicRoi = true;

    // --- Hysteresis Logic ---
    private int lostFrameCount = 0;
    private final int MAX_LOST_FRAMES = 6;

    // --- Debugging Paint System ---
    private final Paint debugPaint = new Paint();

    private final double baseFx = BlackWhiteCamera.cameraMatrix[0];
    private final double baseFy = BlackWhiteCamera.cameraMatrix[4];
    private final double baseCx = BlackWhiteCamera.cameraMatrix[2];
    private final double baseCy = BlackWhiteCamera.cameraMatrix[5];

    private double tagSizeX = 0;
    private double tagSizeY = 0;
    private double tagSizeArea = 0;

    public AprilTagPipeline()
    {
        this.processor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(relativePos, new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + BlackWhiteCamera.pitchAngle, 0, 0))
                .setLensIntrinsics(baseFx, baseFy, baseCx, baseCy)
                .build();

        processor.setDecimation(2);

        matrix.put(0, 0,
                cameraMatrix[0], cameraMatrix[1], cameraMatrix[2],
                cameraMatrix[3], cameraMatrix[4], cameraMatrix[5],
                cameraMatrix[6], cameraMatrix[7], cameraMatrix[8]);

        debugPaint.setColor(Color.GREEN);
        debugPaint.setStyle(Paint.Style.STROKE);
        debugPaint.setStrokeWidth(5.0f);
        debugPaint.setAntiAlias(true);
    }

    public void noteCalibrationIdentity(CameraCalibrationIdentity ident)
    {
        this.ident = ident;
    }

    @Override
    public void init(Mat firstFrame)
    {
        fullBlackCanvas.create(firstFrame.size(), firstFrame.type());

        Mat tempUndistorted = new Mat();
        Calib3d.undistort(firstFrame, tempUndistorted, matrix, dist);
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, tempUndistorted.width(), tempUndistorted.height());
        processor.init(tempUndistorted.width(), tempUndistorted.height(), calibration);
        tempUndistorted.release();
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        // Capture the exact hardware timestamp from the camera thread instantly
        this.latestCaptureTimeNanos = captureTimeNanos;

        // 1. Instantly compute the maximum safe height boundary allowed by our static bottom crop
        int maxAllowedHeight = (int) (input.height() * (1.0 - BOTTOM_CROP_PERCENT));

        // Reset background canvas layout to pure zeroed black space
        fullBlackCanvas.setTo(new Scalar(0));

        boolean processedCropped = false;

        // 2. Adaptive ROI Branch
        if (useDynamicRoi && roiRect != null) {
            int x = Math.max(0, roiRect.x);
            int y = Math.max(0, roiRect.y);
            int width = Math.min(input.width() - x, roiRect.width);

            // Constrain adaptive crop height tightly to prevent spilling into the dead zone
            int height = Math.min(maxAllowedHeight - y, roiRect.height);

            if (width > 40 && height > 40) {
                Rect validBounds = new Rect(x, y, width, height);
                Mat rawCrop = input.submat(validBounds);

                roiMatrix.put(0, 0,
                        baseFx, 0,      baseCx - x,
                        0,      baseFy, baseCy - y,
                        0,      0,      1);

                Calib3d.undistort(rawCrop, undistortedCrop, roiMatrix, dist);
                rawCrop.release();

                Mat canvasSubmat = fullBlackCanvas.submat(validBounds);
                undistortedCrop.copyTo(canvasSubmat);
                canvasSubmat.release();

                processedCropped = true;
            }
        }

        // 3. Full Frame Fallback Branch (Constrained by the permanent bottom crop)
        if (!processedCropped) {
            Rect topRegionRect = new Rect(0, 0, input.width(), maxAllowedHeight);
            Mat rawTopRegion = input.submat(topRegionRect);

            // Re-use roiMatrix to correctly handle the static crop's focal center alignment
            roiMatrix.put(0, 0,
                    baseFx, 0,      baseCx,
                    0,      baseFy, baseCy,
                    0,      0,      1);

            Calib3d.undistort(rawTopRegion, undistortedCrop, roiMatrix, dist);
            rawTopRegion.release();

            Mat canvasSubmat = fullBlackCanvas.submat(topRegionRect);
            undistortedCrop.copyTo(canvasSubmat);
            canvasSubmat.release();
        }

        // 4. Run native AprilTag sweep across the optimized 1280x720 canvas frame layout
        Object drawCtx = processor.processFrame(fullBlackCanvas, captureTimeNanos);
        requestViewportDrawHook(drawCtx);

        updateRoi(maxAllowedHeight);

        return fullBlackCanvas;
    }

    private void updateRoi(int maxAllowedHeight) {
        List<AprilTagDetection> detections = processor.getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            lostFrameCount = 0;

            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = Double.MIN_VALUE;

            for (Point p : tag.corners) {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }

            double w = maxX - minX;
            double h = maxY - minY;

            tagSizeX = w;
            tagSizeY = h;
            tagSizeArea = h * w;

            int padX = (int) (w * (TAG_PADDING_PERCENT + TAG_PADDING_PERCENT_WIDTH));
            int padY = (int) (h * TAG_PADDING_PERCENT);

            int roiX = (int) (minX - padX);
            int roiY = (int) (minY - padY);
            int roiW = (int) (w + (padX * 2));
            int roiH = (int) (h + (padY * 2));

            // Sanity clamp check to make sure our next ROI loop never reaches into the bottom crop zone
            if (roiY + roiH > maxAllowedHeight) {
                roiH = Math.max(0, maxAllowedHeight - roiY);
            }

            roiRect = new Rect(roiX, roiY, roiW, roiH);
        } else {
            lostFrameCount++;
            if (lostFrameCount >= MAX_LOST_FRAMES) {
                roiRect = null;
            }
        }
    }

    public void setUseDynamicRoi(boolean useDynamicRoi) {
        this.useDynamicRoi = useDynamicRoi;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);

        if (roiRect != null) {
            float left = roiRect.x * scaleBmpPxToCanvasPx;
            float top = roiRect.y * scaleBmpPxToCanvasPx;
            float right = (roiRect.x + roiRect.width) * scaleBmpPxToCanvasPx;
            float bottom = (roiRect.y + roiRect.height) * scaleBmpPxToCanvasPx;
            canvas.drawRect(left, top, right, bottom, debugPaint);
        }
    }

    /**
     * Gets the high-precision hardware capture timestamp of the frame containing the current detections.
     * @return System.nanoTime() baseline from the OS kernel driver layer.
     */
    public long getLatestTimestamp() {
        return latestCaptureTimeNanos;
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
        double tagFieldY =  tagfieldPose.getY();

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

    public Pose getPose() {
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) return new Pose(0, 0, 0);

        AprilTagDetection detection = detections.get(0);
        if (detection.robotPose == null) return new Pose(0, 0, 0);

        // 1. Force the SDK to return position in INCHES to match Pedro Pathing
        Position pose = detection.robotPose.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orientation = detection.robotPose.getOrientation();

        // 2. Construct using FTC coordinates, then let Pedro convert it safely
        return new Pose(
                pose.y + 72,
                -pose.x + 72,
                orientation.getYaw(AngleUnit.RADIANS)
        );
    }

    public double getTagSizeArea() {
        return tagSizeArea;
    }

    public double getTagSizeY() {
        return tagSizeY;
    }

    public double getTagSizeX() {
        return tagSizeX;
    }
}