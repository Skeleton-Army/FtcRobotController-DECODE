package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.vision.LimelightArtifact;


@TeleOp
public class PickUpArtifact extends OpMode {
    private Follower follower;
    LimelightArtifact limeLightArtifact;
    Drive drive;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(180)));

        limeLightArtifact = new LimelightArtifact(hardwareMap, follower);

        drive = new Drive(follower, Alliance.RED);
        drive.setDefaultCommand(new RunCommand(
                () -> drive.teleOpDrive(gamepad1), drive
        ));
    }

    @Override
    public void loop() {
        follower.update();
        //CommandScheduler.getInstance().run();
        Pose artifactFieldPos = limeLightArtifact.getClosestArtifact();
        telemetry.addData("artifact x", artifactFieldPos.getX());
        telemetry.addData("artifact y", artifactFieldPos.getX());
        telemetry.addData("robot x", follower.getPose().getX());
        telemetry.addData("robot y", follower.getPose().getY());
        telemetry.addData("robot heading", follower.getHeading());

        // idk implement going to it ig
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
