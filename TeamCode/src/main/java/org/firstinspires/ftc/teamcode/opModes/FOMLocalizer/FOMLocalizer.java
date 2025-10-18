package org.firstinspires.ftc.teamcode.opModes.FOMLocalizer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class FOMLocalizer extends OpMode {
    Follower follower;
    Limelight3A limelight;

    TelemetryPacket telemetryPacket;
    FtcDashboard dashboard;


    // starting values
    double odometryFOM = 0.01;
    double apriltagFOM = 1;



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        telemetryPacket = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();


    }

    @Override
    public void loop() {

    }
}
