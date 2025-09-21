package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.logging.Logger;

@TeleOp(name = "limelight apriltag")
public class LimelightToAdvantageScope extends OpMode {


    TelemetryPacket telemetryPacket;
    FtcDashboard dashboard;
    private Limelight3A limelight;
    Telemetry dashboardTelemetry;
    @Override
    public void init() {
        telemetryPacket = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();

            telemetryPacket.put("Pose x", botpose.getPosition().x);
            telemetryPacket.put("Pose y", botpose.getPosition().y);
            telemetryPacket.put("Pose z", botpose.getPosition().z);
            telemetryPacket.put("Pose heading", botpose.getOrientation().getYaw());

            dashboardTelemetry.addData("Pose x", botpose.getPosition().x);
            dashboardTelemetry.addData("Pose y", botpose.getPosition().y);
            dashboardTelemetry.addData("Pose z", botpose.getPosition().z);
            dashboardTelemetry.addData("Pose heading", botpose.getOrientation().getYaw());

            telemetry.addData("Pose x", botpose.getPosition().x);
            telemetry.addData("Pose y", botpose.getPosition().y);
            telemetry.addData("Pose Z", botpose.getPosition().z);
        }

        telemetryPacket.put("Cpu", status.getCpu());
        telemetryPacket.put("Ram", status.getRam());
        telemetryPacket.put("result: ", result);

        telemetry.update();
        dashboardTelemetry.update();
        dashboard.sendTelemetryPacket(telemetryPacket);

    }
}
