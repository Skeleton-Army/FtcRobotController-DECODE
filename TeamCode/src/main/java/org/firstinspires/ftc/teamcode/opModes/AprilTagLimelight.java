package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp()
public class AprilTagLimelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight_webcam");
        limelight.pipelineSwitch(3); //April tag pipeline is 3
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight.start();

    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            telemetry.addData("X relative: ",llResult.getTx());
            telemetry.addData("Y relative: ",llResult.getTy());
            Pose3D botPose = llResult.getBotpose_MT2();
            //botPose.getPosition().toUnit(DistanceUnit.INCH);
            telemetry.addData("y", botPose.getPosition().toUnit(DistanceUnit.INCH).y);
            telemetry.addData("x", botPose.getPosition().toUnit(DistanceUnit.INCH).x);
        }
    }
}
