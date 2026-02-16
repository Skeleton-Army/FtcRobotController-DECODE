package org.firstinspires.ftc.teamcode.opModes.testsVision;

import static org.firstinspires.ftc.teamcode.config.LimelightConfig.ARTIFACT_HEIGHT_FROM_FLOOR;
import static org.firstinspires.ftc.teamcode.config.LimelightConfig.LENS_HEIGHT_INCHES;
import static org.firstinspires.ftc.teamcode.config.LimelightConfig.LIMELIGHT_MOUNT_ANGLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.config.LimelightConfig;

// This OpMode doesn't use the `LimelightArtifact` class on purpose, it's used for debugging the calibration.
@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    Follower follower;
    Drive drive;
    private Pose fieldPos; // Field-relative position of the sample, uhhhh i mean pixel

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setPose(new Pose(72,72));
        follower.setMaxPower(1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LimelightConfig.PIPELINE_INDEX);
        // 0: apriltag detection
        // 1: artifact detection
        // 2: test pipeline
        limelight.start();

        drive = new Drive(follower, pickRandomAlliance());
        drive.setDefaultCommand(new RunCommand(
                () -> drive.teleOpDrive(gamepad1), drive
        ));
    }

    @Override
    public void loop() {
        follower.update();

        //Uncomment for driving
        //CommandScheduler.getInstance().run();

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null) {

            double[] llPytohn = llResult.getPythonOutput();

            Pose robotPose = follower.getPose();
            double artifactDistance = getDistance(llPytohn[1]);
            Pose artifactPosition = getRelativePos(artifactDistance,llPytohn[0]);
            fieldPos = calculateField(robotPose, artifactPosition);

            telemetry.addData("tx ", llPytohn[0]);
            telemetry.addData("ty ", llPytohn[1]);
            telemetry.addData("Artifact Distance ", artifactDistance);
            telemetry.addData("Artifact Pos X", artifactPosition.getX());
            telemetry.addData("Artifact Pos Y", artifactPosition.getY());
            telemetry.addData("Artifact Field Pos X", fieldPos.getX());
            telemetry.addData("Artifact Field Pos Y", fieldPos.getY());
            telemetry.addData("Robot X ", follower.getPose().getX());
            telemetry.addData("Robot Y", follower.getPose().getY());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }


    private Alliance pickRandomAlliance() {
        if (Math.random() >= .5) {
            return Alliance.BLUE;
        }  else {
            return Alliance.RED;
        }
    }
    public static double getDistance(double ty){

        double angleToGoalRadians = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + ty);
        //calculate distance
        return (ARTIFACT_HEIGHT_FROM_FLOOR - LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
    }

    // Relative to Sholef 2
    public static Pose getRelativePos( double artifactDistance, double tx) {
        double thetaRad = Math.toRadians(tx);

        // Calculate offsets
        double deltaX = (artifactDistance + LimelightConfig.X_OFFSET_INCHES) * Math.cos(thetaRad);
        double deltaY = (artifactDistance + LimelightConfig.Y_OFFSET_INCHES) * Math.sin(thetaRad);

        return new Pose(deltaX, deltaY);
    }

    public static Pose calculateField (Pose robotPose, Pose artifactRelative){
        double theta = robotPose.getHeading();
        double yOffset = artifactRelative.getX() * Math.sin(theta) - artifactRelative.getY() * Math.sin(theta);
        double xOffset = artifactRelative.getX() * Math.cos(theta) - artifactRelative.getY() * Math.cos(theta);
        return new Pose(robotPose.getX() + xOffset, robotPose.getY() + yOffset);
    }

    public static Pose getArtifactAbsolutePos(Pose robotPose, double[] llPython) {
        double tx = llPython[0], ty = llPython[1];
        Pose artifactPosition = getRelativePos(getDistance(ty),tx);
        return calculateField(robotPose, artifactPosition);

    }

}
