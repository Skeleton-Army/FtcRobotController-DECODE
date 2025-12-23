package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    Follower follower;
    // Hc: Height of the Limelight lens from the floor (e.g., 0.35m or ~13.78 inches)


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
            double[] ArtifactFieldPosition = getFieldPosition(ArtifactDistance,llResult.getPythonOutput(),robotPose);

            telemetry.addData("tx: ", llPytohn[0]);
            telemetry.addData("ty: ", llPytohn[1]);
            telemetry.addData("ArtifactX", ArtifactFieldPosition[0]);
            telemetry.addData("ArtifactY", ArtifactFieldPosition[1]);

        }
        telemetry.update();
    }
    public double getDistance(double[] llPyOut, double limelightMountAngleDegrees, double goalHeightInches, double limelightLensHeightInches){
        double targetOffsetAngle_Vertical = llPyOut[1];

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);
        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public static double[] getFieldPosition(
            double ArtifactDistance,
            double[] anglesDeg,
            Pose robotPose
    ) {
        // Convert degrees to radians
        double thetaRad = Math.toRadians(anglesDeg[0]);

        // Calculate offsets
        double deltaX = (ArtifactDistance + ArtifactTrackingConfig.X_OFFSET - ArtifactTrackingConfig.Y_OFFSET) * Math.cos(thetaRad);
        double deltaY = ArtifactDistance * Math.sin(thetaRad);

        // Field position
        double fieldX = robotPose.getX() + deltaX;
        double fieldY = robotPose.getY() + deltaY;

        return new double[] { fieldX, fieldY };
    }




    @Override
    public void stop() {
        limelight.stop();
    }
}
