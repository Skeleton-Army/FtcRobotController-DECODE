package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.logging.Logger;

@TeleOp(name = "limelight apriltag", group = "limelight")
public class LimelightToAdvantageScope extends OpMode {


    TelemetryPacket telemetryPacket;
    FtcDashboard dashboard;
    private Limelight3A limelight;
    private Follower follower;
    @Override
    public void init() {
        telemetryPacket = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

    }

    @Override
    public void loop() {
        // enabling driver control while detecting apriltag
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // robot's movement stats
        telemetryPacket.put("velocity", follower.getVelocity());
        telemetryPacket.put("acceleration", follower.getAcceleration());

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles orientation = result.getBotpose().getOrientation();

            telemetryPacket.put("Pose x", botpose.x);
            telemetryPacket.put("Pose y", botpose.y);
            telemetryPacket.put("Pose z", botpose.z);
            telemetryPacket.put("Pose heading", orientation.getYaw());

            telemetry.addData("Pose x", botpose.x);
            telemetry.addData("Pose y", botpose.y);
            telemetry.addData("Pose Z", botpose.z);
            telemetryPacket.put("Pose heading", orientation.getYaw());
        }

        // limelights stats
        telemetryPacket.put("Cpu", status.getCpu());
        telemetryPacket.put("Ram", status.getRam());
        telemetryPacket.put("Ram", status.getFps());

        telemetry.update();
        dashboard.sendTelemetryPacket(telemetryPacket);

    }
}
