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
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

public class AprilTagPipeline extends TimestampedOpenCvPipeline
{
    private AprilTagProcessor processor;
    private CameraCalibrationIdentity ident;

    Mat undistorted = new Mat();
    MatOfDouble dist = new MatOfDouble(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4]);
    Mat matrix = new Mat(3, 3, CvType.CV_64F);

    // Custom camera matrix to track shifting center points in cropped frame
    Mat roiMatrix = new Mat(3, 3, CvType.CV_64F);

    // --- Dynamic ROI Variables ---
    private Rect roiRect = null;
    private final double ROI_PADDING_FACTOR = 1.2; // 120% padding to prevent tracking dropouts during rapid motion
    private boolean useDynamicRoi = true;

    // Cache native camera intrinsics
    private final double baseFx = BlackWhiteCamera.cameraMatrix[0];
    private final double baseFy = BlackWhiteCamera.cameraMatrix[4];
    private final double baseCx = BlackWhiteCamera.cameraMatrix[2];
    private final double baseCy = BlackWhiteCamera.cameraMatrix[5];

    public AprilTagPipeline()
    {
        this.processor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90 + BlackWhiteCamera.pitchAngle, 0, 0))
                .setLensIntrinsics(baseFx, baseFy, baseCx, baseCy)
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
        Calib3d.undistort(firstFrame, undistorted, matrix, dist);
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, undistorted.width(), undistorted.height());
        processor.init(undistorted.width(), undistorted.height(), calibration);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Mat cropMat = input;
        int offsetX = 0;
        int offsetY = 0;

        // 1. Crop BEFORE Undistortion
        if (useDynamicRoi && roiRect != null) {
            int x = Math.max(0, roiRect.x);
            int y = Math.max(0, roiRect.y);
            int width = Math.min(input.width() - x, roiRect.width);
            int height = Math.min(input.height() - y, roiRect.height);

            if (width > 20 && height > 20) {
                cropMat = input.submat(new Rect(x, y, width, height));
                offsetX = x;
                offsetY = y;
            }
        }

        // 2. Adjust calibration matrix dynamically for the cropped region
        // Shift principal center (cx, cy) by our crop offset; keep focal length (fx, fy) identical
        roiMatrix.put(0, 0,
                baseFx, 0,      baseCx - offsetX,
                0,      baseFy, baseCy - offsetY,
                0,      0,      1);

        // 3. Undistort only the isolated cropped region
        Calib3d.undistort(cropMat, undistorted, roiMatrix, dist);

        // 4. Update processor internally to adapt to changing submat dimensions
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, undistorted.width(), undistorted.height());
        processor.init(undistorted.width(), undistorted.height(), calibration);

        // 5. Run standard AprilTag detection logic
        Object drawCtx = processor.processFrame(undistorted, captureTimeNanos);
        requestViewportDrawHook(drawCtx);

        // 6. Project tracked bounds out for next frame tracking loop
        updateRoi(cropMat.width() == input.width(), offsetX, offsetY);

        // Clean up submat reference allocations
        if (cropMat != input) {
            cropMat.release();
        }

        // Return the modified frame. If you want to visually verify the crop window
        // on the Driver Station or FTC Dashboard, change this line to: return undistorted;
        return input;
    }

    private void updateRoi(boolean processedFullFrame, int currentOffsetX, int currentOffsetY) {
        java.util.List<AprilTagDetection> detections = processor.getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            // Re-map localized crop-space corner detections back to absolute canvas pixels
            double absoluteMinX = Double.MAX_VALUE;
            double absoluteMaxX = Double.MIN_VALUE;
            double absoluteMinY = Double.MAX_VALUE;
            double absoluteMaxY = Double.MIN_VALUE;

            for (Point p : tag.corners) {
                double absX = p.x + currentOffsetX;
                double absY = p.y + currentOffsetY;
                if (absX < absoluteMinX) absoluteMinX = absX;
                if (absX > absoluteMaxX) absoluteMaxX = absX;
                if (absY < absoluteMinY) absoluteMinY = absY;
                if (absY > absoluteMaxY) absoluteMaxY = absY;
            }

            double tagWidth = absoluteMaxX - absoluteMinX;
            double tagHeight = absoluteMaxY - absoluteMinY;

            // Apply scaling padding boundaries
            int paddingX = (int) (tagWidth * ROI_PADDING_FACTOR);
            int paddingY = (int) (tagHeight * ROI_PADDING_FACTOR);

            int roiX = (int) (absoluteMinX - paddingX);
            int roiY = (int) (absoluteMinY - paddingY);
            int roiW = (int) (tagWidth + (paddingX * 2));
            int roiH = (int) (tagHeight + (paddingY * 2));

            roiRect = new Rect(roiX, roiY, roiW, roiH);
        } else {
            // Target lost, scan full frame next loop
            roiRect = null;
        }
    }

    public void setUseDynamicRoi(boolean useDynamicRoi) {
        this.useDynamicRoi = useDynamicRoi;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }

    public Pose getRobotPose(double turretAngle) {
        if (!processor.getDetections().isEmpty()) {
            AprilTagDetection detection = processor.getDetections().get(0);
            double relX = detection.ftcPose.x;
            double relY = detection.ftcPose.y;
            double relHeading = Math.toRadians(detection.ftcPose.bearing);

            double cosT = Math.cos(turretAngle);
            double sinT = Math.sin(turretAngle);

            double rotatedX = relX * cosT - relY * sinT;
            double rotatedY = relX * sinT + relY * cosT;

            double camOffsetX = offsetX_turret;
            double camOffsetY = offsetY_turret;

            double robotX = rotatedX - camOffsetX;
            double robotY = rotatedY - camOffsetY;

            double robotHeading = relHeading + turretAngle;

            return new Pose(robotX, robotY, robotHeading);
        }
        return new Pose(0,0,0);
    }

    public Pose getPedroPose(double robotHeading, double turretHeading) {
        if (processor.getDetections().isEmpty())
            return new Pose(0,0,0);

        AprilTagDetection detection = processor.getDetections().get(0);
        if (detection.metadata == null) {
            return new Pose(0,0,0);
        }

        Pose2D tagfieldPos2D = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0);
        if (detection.id == 20) {
            tagfieldPos2D = new Pose2D(DistanceUnit.METER, -1.482, -1.413, AngleUnit.RADIANS, 0);
        }

        if (detection.id == 24) {
            tagfieldPos2D = new Pose2D(DistanceUnit.METER,-1.482, 1.413,AngleUnit.RADIANS, 0);
        }

        Pose ftcStandard = PoseConverter.pose2DToPose(tagfieldPos2D, InvertedFTCCoordinates.INSTANCE);
        Pose tagfieldPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        double tagFieldX = tagfieldPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE).getX();
        double tagFieldY =  tagfieldPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE).getY();

        double localForwardDist = detection.ftcPose.y;
        double localLeftDist = -detection.ftcPose.x;

        double thetaCam = robotHeading + turretHeading;

        double fieldDistX = (localForwardDist * Math.cos(thetaCam)) - (localLeftDist * Math.sin(thetaCam));
        double fieldDistY = (localForwardDist * Math.sin(thetaCam)) + (localLeftDist * Math.cos(thetaCam));

        double camFieldX = tagFieldX - fieldDistX;
        double camFieldY = tagFieldY - fieldDistY;

        double offsetFieldX = (offsetY_turret * Math.cos(robotHeading)) - (offsetX_turret * Math.sin(robotHeading));
        double offsetFieldY = (offsetY_turret * Math.sin(robotHeading)) + (offsetX_turret * Math.cos(robotHeading));

        double robotFieldX = camFieldX - offsetFieldX;
        double robotFieldY = camFieldY - offsetFieldY;

        return new Pose(robotFieldX, robotFieldY, robotHeading);
    }

    public AprilTagDetection getApriltagDetection() {
        if (!processor.getDetections().isEmpty())
            return processor.getDetections().get(0);
        return null;
    }

    public AprilTagProcessor getProcessor() {
        return processor;
    }
}