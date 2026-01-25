package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.calculators.LimeLightCalculator.calculateField;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calculators.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.ArtifactDetection;
import org.psilynx.psikit.core.wpi.math.Pose2d;


@TeleOp
public class PickUpArtifact extends OpMode {
    private Follower follower;
    private Pose2d fieldPos;
    ArtifactDetection detection;
    Limelight3A limelight;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 180));


    }

    @Override
    public void loop() {
        follower.update(); // Always call this first!


        if (limelight.getLatestResult() != null) {
            // Replace these with your actual vision variables
            double x = detection.getX();//Artifact field X
            double y = detection.getY();//Artifact field Y

            updateArtifactFollow(x, y);
        } else {
            follower.holdPoint(follower.getPose());
        }
    }
    // Inside your OpMode loop
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

}
