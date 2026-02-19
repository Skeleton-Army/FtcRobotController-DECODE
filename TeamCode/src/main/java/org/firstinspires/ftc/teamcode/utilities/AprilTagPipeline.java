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
        Calib3d.undistort(firstFrame, undistored, matrix, dist);
        CameraCalibration calibration = CameraCalibrationHelper.getInstance().getCalibration(ident, undistored.width(), undistored.height());
        processor.init(undistored.width(), undistored.height(), calibration);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Calib3d.undistort(input, undistored, matrix, dist);
        Object drawCtx = processor.processFrame(undistored, captureTimeNanos);
        requestViewportDrawHook(drawCtx);
        return undistored;
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
            double camOffsetX = offsetX_turret;
            double camOffsetY = offsetY_turret;

            // Translate to robot center
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

        // 1. We MUST have the tag's known position on the field to find ourselves
        if (detection.metadata == null) {
            return new Pose(0,0,0);
        }

        // (Assuming you have a helper or constants that define the tag's field location in Pedro coordinates)
        // You will need to replace these with however you store your Tag field positions
//        double tagFieldX = getTagFieldX(detection.id);
        Pose2D tagfieldPos2D = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0);
        if (detection.id == 20) {
            tagfieldPos2D = new Pose2D(DistanceUnit.METER, -1.482, -1.413, AngleUnit.RADIANS, 0);
        }

        if (detection.id == 24) {
            tagfieldPos2D = new Pose2D(DistanceUnit.METER,-1.482, 1.413,AngleUnit.RADIANS, 0);
        }

        Pose ftcStandard = PoseConverter.pose2DToPose(tagfieldPos2D, InvertedFTCCoordinates.INSTANCE);
        Pose tagfieldPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        //Pose tagfieldPose = new Pose(detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1), 0);
        //tagfieldPose = tagfieldPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        //OpModeManager.getTelemetry().addData("tag field pose: ", tagfieldPose);

        OpModeManager.getTelemetry().addData("tag field pose pedro: ", tagfieldPose);
        double tagFieldX = tagfieldPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE).getX();
//        double tagFieldY = getTagFieldY(detection.id);
        double tagFieldY =  tagfieldPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE).getY();

        OpModeManager.getTelemetry().addData("tag x", tagFieldX);
        OpModeManager.getTelemetry().addData("tag y", tagFieldY);
        // 2. Extract relative distances from camera to tag
        // FTC SDK ftcPose: +Y is forward (depth), +X is right.
        // Pedro standard: +X is forward, +Y is left.
        // So we map them to Pedro's local orientation:
        double localForwardDist = detection.ftcPose.y;
        double localLeftDist = -detection.ftcPose.x;

        // 3. Camera's global heading on the field
        double thetaCam = robotHeading + turretHeading;

        // 4. Rotate the Camera-to-Tag distance vector to match the field frame
        double fieldDistX = (localForwardDist * Math.cos(thetaCam)) - (localLeftDist * Math.sin(thetaCam));
        double fieldDistY = (localForwardDist * Math.sin(thetaCam)) + (localLeftDist * Math.cos(thetaCam));

        // 5. Calculate Camera's absolute position on the field
        // Camera is exactly the Tag's position MINUS the distance to the tag
        double camFieldX = tagFieldX - fieldDistX;
        double camFieldY = tagFieldY - fieldDistY;

        // 6. Rotate the Robot-to-Turret offset to match the field frame
        // (Assuming CAMERA_OFFSET_X is forward, CAMERA_OFFSET_Y is left)
        double offsetFieldX = (offsetY_turret * Math.cos(robotHeading)) - (offsetX_turret * Math.sin(robotHeading));
        double offsetFieldY = (offsetY_turret * Math.sin(robotHeading)) + (offsetX_turret * Math.cos(robotHeading));

        // 7. Calculate Robot's absolute position
        // Robot center is exactly the Camera's position MINUS the offset
        double robotFieldX = camFieldX - offsetFieldX;
        double robotFieldY = camFieldY - offsetFieldY;

        // Return the final Pedro Pose using the Pinpoint's highly accurate heading
        return new Pose(robotFieldX, robotFieldY, robotHeading);
    }


    public AprilTagDetection getApriltagDetection() {
        if (!processor.getDetections().isEmpty())
            return processor.getDetections().get(0);
        return null;
    }

}
