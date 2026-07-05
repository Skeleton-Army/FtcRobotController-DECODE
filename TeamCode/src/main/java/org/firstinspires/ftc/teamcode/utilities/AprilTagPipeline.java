package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.DECIMATION_FALLBACK;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.DECIMATION_LOCKED;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.blueTagBiasX;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.blueTagBiasY;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.cameraMatrix;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.distCoeffs;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.offsetX_turret;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.offsetY_turret;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.redTagBiasX;
import static org.firstinspires.ftc.teamcode.config.BlackWhiteCamera.redTagBiasY;
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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
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

    // --- Precomputed Undistort Maps ---
    private Mat mapX_full = new Mat();
    private Mat mapY_full = new Mat();

    // --- High-Precision Timing Anchor ---
    private volatile long latestCaptureTimeNanos = 0;

    // --- Fallback-frame tracking ---
    // The full-frame fallback branch runs on non-undistorted pixels (cheap, and fine
    // for re-acquiring the ROI), so poses computed on it carry distortion bias and
    // are never published as a snapshot (see below). The filter just rides on
    // odometry until the ROI branch locks back on.
    private volatile boolean lastFrameWasFallback = false;

    /**
     * Immutable pairing of a trusted robot pose and the hardware capture timestamp of
     * the frame it was computed from. Published atomically once per processed frame.
     */
    public static class PoseSnapshot {
        public final Pose pose;
        public final long timestampNanos;

        PoseSnapshot(Pose pose, long timestampNanos) {
            this.pose = pose;
            this.timestampNanos = timestampNanos;
        }
    }

    private volatile PoseSnapshot latestSnapshot = null;

    // --- Permanent Static Crop ---
    private final double BOTTOM_CROP_PERCENT = 0.20; // Cuts off the bottom 20% of the image entirely

    // --- Dynamic Adaptive ROI Bounds ---
    private Rect roiRect = null;
    public static double TAG_PADDING_PERCENT = 0.5;
    public static double TAG_PADDING_PERCENT_WIDTH = 1;
    private boolean useDynamicRoi = true;

    // --- Hysteresis Logic ---
    private int lostFrameCount = 0;

    // --- Adaptive Decimation ---
    private float currentDecimation = -1f; // force-set on first frame

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
                .setDrawCubeProjection(true)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(relativePos, new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + BlackWhiteCamera.pitchAngle, 0, 0))
//                .setCameraPose(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + BlackWhiteCamera.pitchAngle, 0, 0))
                .setLensIntrinsics(baseFx, baseFy, baseCx, baseCy)
                .build();

        setDecimation(DECIMATION_LOCKED);

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

        // Build the full-resolution undistort map ONCE. This map is reused every frame
        // for both the ROI branch (sliced) and the full-frame fallback branch (sliced to the top crop).
        Calib3d.initUndistortRectifyMap(matrix, dist, new Mat(), matrix, firstFrame.size(), CvType.CV_32FC1, mapX_full, mapY_full);

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

                Mat mapXRoi = mapX_full.submat(validBounds);
                Mat mapYRoi = mapY_full.submat(validBounds);

                Imgproc.remap(input, undistortedCrop, mapXRoi, mapYRoi, Imgproc.INTER_LINEAR);

                mapXRoi.release();
                mapYRoi.release();

                Mat canvasSubmat = fullBlackCanvas.submat(validBounds);
                undistortedCrop.copyTo(canvasSubmat);
                canvasSubmat.release();

                processedCropped = true;
            }
        }

        // 3. Full Frame Fallback Branch (Constrained by the permanent bottom crop)
        // NOTE: intentionally skips undistort/remap here. Distortion correction only matters for pose accuracy.
        if (!processedCropped) {
            Rect topRegionRect = new Rect(0, 0, input.width(), maxAllowedHeight);
            Mat rawTopRegion = input.submat(topRegionRect);
            Mat canvasSubmat = fullBlackCanvas.submat(topRegionRect);
            rawTopRegion.copyTo(canvasSubmat);
            rawTopRegion.release();
            canvasSubmat.release();
        }

        lastFrameWasFallback = !processedCropped;

        setDecimation(processedCropped ? DECIMATION_LOCKED : DECIMATION_FALLBACK);

        // 4. Run native AprilTag sweep across the optimized 1280x720 canvas frame layout
        Object drawCtx = processor.processFrame(fullBlackCanvas, captureTimeNanos);
        requestViewportDrawHook(drawCtx);

        updateRoi(maxAllowedHeight);

        this.latestCaptureTimeNanos = captureTimeNanos;

        Pose framePose = processedCropped ? computePose() : null;
        this.latestSnapshot = (framePose != null) ? new PoseSnapshot(framePose, captureTimeNanos) : null;

        return fullBlackCanvas;
    }

    private void setDecimation(float decimation) {
        if (currentDecimation != decimation) {
            processor.setDecimation(decimation);
            currentDecimation = decimation;
        }
    }

    private void updateRoi(int maxAllowedHeight) {
        List<AprilTagDetection> detections = getFilteredDetections();

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
            if (lostFrameCount >= BlackWhiteCamera.MAX_LOST_FRAMES) {
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

    /** True if the most recent processed frame went through the full-frame fallback branch rather than the locked ROI branch. */
    public boolean wasLastFrameFallback() {
        return lastFrameWasFallback;
    }

    public List<AprilTagDetection> getDetections() {
        return processor.getDetections();
    }

    private List<AprilTagDetection> getFilteredDetections() {
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null) return null;

        List<AprilTagDetection> filtered = new java.util.ArrayList<>();
        for (AprilTagDetection d : detections) {
            if (d.id == 20 || d.id == 24) {
                filtered.add(d);
            }
        }
        return filtered;
    }

    public AprilTagDetection getApriltagDetection() {
        // Never hand out a detection computed on an un-undistorted fallback frame —
        // its pose is bias-corrupted. Wait for the ROI branch to lock back on.
        if (lastFrameWasFallback) return null;

        List<AprilTagDetection> detections = getFilteredDetections();
        return (detections != null && !detections.isEmpty()) ? detections.get(0) : null;
    }

    public AprilTagProcessor getProcessor() {
        return processor;
    }

    /**
     * Atomic (pose, capture-timestamp) pair from the most recent trusted frame, or
     * null if the latest frame had no valid detection or was a fallback frame.
     */
    public PoseSnapshot getPoseSnapshot() {
        return latestSnapshot;
    }

    /** Computes the robot pose from current detections, or null if unavailable. */
    private Pose computePose() {
        List<AprilTagDetection> detections = getFilteredDetections();
        if (detections == null || detections.isEmpty()) return null;

        AprilTagDetection detection = detections.get(0);
        if (detection.robotPose == null) return null;

        // 1. Force the SDK to return position in INCHES to match Pedro Pathing
        Position pose = detection.robotPose.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orientation = detection.robotPose.getOrientation();
        double heading = orientation.getYaw(AngleUnit.RADIANS);

        // Per-tag bias, defined in robot-relative frame: X = left/right, Y = forward/back
        double biasX = (detection.id == 20) ? blueTagBiasX : redTagBiasX;
        double biasY = (detection.id == 20) ? blueTagBiasY : redTagBiasY;

        // Rotate robot-relative bias into field-relative X/Y using current heading
        double fieldBiasX = (biasY * Math.cos(heading)) - (biasX * Math.sin(heading));
        double fieldBiasY = (biasY * Math.sin(heading)) + (biasX * Math.cos(heading));

        // 2. Construct using FTC coordinates, then let Pedro convert it safely
        return new Pose(
                pose.y + 72 + fieldBiasX,
                -pose.x + 72 + fieldBiasY,
                heading
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