package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.calculators.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.vision.ArtifactTrackingConfig;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.util.Arrays;


@TeleOp
public class PickUpArtifact extends OpMode {
    private Follower follower;
    private Pose artifactFieldPos;
    private Pose relativePose;
    Limelight3A limelight;
    Drive drive;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(180)));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ArtifactTrackingConfig.PIPELINE_INDEX);

        drive = new Drive(follower, Alliance.RED);
        drive.setDefaultCommand(new RunCommand(
                () -> drive.teleOpDrive(gamepad1), drive
        ));
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();
        //CommandScheduler.getInstance().run();


        LLResult llResult = limelight.getLatestResult();
        if (llResult != null) {
            double[] llPython = llResult.getPythonOutput();
            artifactFieldPos = LimeLightCalculator.getArtifactAbsolutePos(follower.getPose(),llPython);
            double distance = LimeLightCalculator.getDistance(llPython[1]);
            relativePose = LimeLightCalculator.getRelativePos(distance, llPython[0]);
            Path path = new Path(
                    new BezierLine(
                            follower.getPose(),
                            artifactFieldPos
                    )
            );
            //follower.followPath(path);
            telemetry.addData("llpython", Arrays.toString(llPython));
            telemetry.addData("Tx", llPython[0]);
            telemetry.addData("Ty", llPython[1]);
            telemetry.addData("Distance", distance);
            telemetry.addData("RelativeX", relativePose.getX());
            telemetry.addData("RelativeY", relativePose.getY());
            telemetry.addData("filedX", artifactFieldPos.getX());
            telemetry.addData("filedY", artifactFieldPos.getY());
        }
    }

    /* idk what does this method does
    public void updateArtifactFollow(double artifactX, double artifactY) {
        // 1. Get current robot pose
        Pose currentPose = follower.getPose();

        // 2. Calculate Distance Error (to check if we are already close enough)
        double distanceError = Math.hypot(artifactX - currentPose.getX(), artifactY - currentPose.getY());

        // 3. Set a Deadzone (e.g., 1.5 inches)
        // This prevents jittering when the robot is basically touching the target.
        if (distanceError < 1.5) {
            follower.holdPoint(follower.getPose()); // Stops the robot smoothly
            return;
        }

        // 4. Calculate Heading Error (so the front of the robot faces the artifact)
        double deltaX = artifactX - currentPose.getX();
        double deltaY = artifactY - currentPose.getY();
        double targetHeading = Math.atan2(deltaY, deltaX);

        // 5. Create the Target Pose
        Pose targetPose = new Pose(artifactX, artifactY, currentPose.getHeading() + targetHeading);

        // 6. Command the Follower
        // Parameters: (Target Pose, isFieldCentric)
        // We set isFieldCentric to 'true' because your coordinates are (0-144) field positions.
        follower.holdPoint(targetPose, true);
    }
     */
}
