package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.calculators.LimeLightCalculator.calculateField;
import static org.firstinspires.ftc.teamcode.calculators.LimeLightCalculator.getDistance;
import static org.firstinspires.ftc.teamcode.calculators.LimeLightCalculator.getRelativePos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.psilynx.psikit.core.wpi.math.Pose2d;

@TeleOp
public class ArtifactDetection extends OpMode {
    Limelight3A limelight;
    Follower follower;
    Drive drive;
    private Pose2d fieldPos; // Field-relative position of the sample, uhhhh i mean pixel




    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setPose(new Pose(72,72));
        follower.setMaxPower(1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ArtifactTrackingConfig.PIPELINE_INDEX);
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
            Pose2d artifactPosition = getRelativePos(artifactDistance,llPytohn[0]);
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
    public double getX() { return fieldPos.getX(); }
    public double getY() { return fieldPos.getY(); }

}
