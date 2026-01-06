package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.ROB_X;
import static org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig.ROB_Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    Follower follower;
    // Hc: Height of the Limelight lens from the floor (e.g., 0.35m or ~13.78 inches)
    private Pose2d fieldPos; // Field-relative position of the sample



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ArtifactTrackingConfig.PIPELINE_INDEX);
        // 0: apriltag detection
        // 1: artifact detection
        // 2: test pipeline
        limelight.start();



    }

    @Override
    public void loop() {
        follower.update();

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null) {


            double[] llPytohn = llResult.getPythonOutput();

            double ArtifactDistance = getDistance(llPytohn,ArtifactTrackingConfig.LIMELIGHT_MOUNT_ANGLE,ArtifactTrackingConfig.ARTIFACT_HEIGHT_FROM_FLOOR,ArtifactTrackingConfig.LENS_HEIGHT_INCHES);
            Pose robotPose = follower.getPose();
            Pose2d ArtifactPosition = getRelativePos(ArtifactDistance,llResult.getPythonOutput(),robotPose);
            calculateField(robotPose, ArtifactPosition.getX(), ArtifactPosition.getY());


            telemetry.addData("tx: ", llPytohn[0]);
            telemetry.addData("ty: ", llPytohn[1]);
            telemetry.addData("ArtifactX", ArtifactPosition.getX());
            telemetry.addData("ArtifactY", ArtifactPosition.getY());
            telemetry.addData("FieldX", fieldPos.getX());
            telemetry.addData("FieldY", fieldPos.getY());
            telemetry.addData("Robot X ", ROB_X);
            telemetry.addData("Robot Y", ROB_Y);

        }
        telemetry.update();
    }
    public double getDistance(double[] llPyOut, double limelightMountAngleDegrees, double goalHeightInches, double limelightLensHeightInches){
        double targetOffsetAngle_Vertical = llPyOut[1];

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public static Pose2d getRelativePos(double ArtifactDistance, double[] anglesDeg, Pose robotPose) {
        // Convert degrees to radians
        double thetaRad = Math.toRadians(anglesDeg[0]);

        // Calculate offsets
        double deltaX = (ArtifactDistance + ArtifactTrackingConfig.X_OFFSET - ArtifactTrackingConfig.Y_OFFSET) * Math.cos(thetaRad);
        double deltaY = ArtifactDistance * Math.sin(thetaRad);

        // Field position
        double fieldX = ROB_X + deltaX;
        double fieldY = ROB_Y + deltaY;

        return new Pose2d(fieldX, fieldY, new Rotation2d(0));
    }

    public void calculateField(Pose robotPose, double centerX, double  centerY) {
        double x = robotPose.getX() + centerY * Math.cos(robotPose.getHeading()) - centerX * Math.sin(robotPose.getHeading());
        double y = robotPose.getY() + centerY * Math.sin(robotPose.getHeading()) + centerX * Math.cos(robotPose.getHeading());
        fieldPos = new Pose2d(x, y, new Rotation2d(0));
    }



    @Override
    public void stop() {
        limelight.stop();
    }
}
