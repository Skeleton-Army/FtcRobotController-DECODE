package org.firstinspires.ftc.teamcode.opModes;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;

@TeleOp(name = "Psikit - OpMode")
public class teleop_psikit extends OpMode {

    Follower follower;
    Limelight3A limelight;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setPose(new Pose(0,0,0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        RLOGServer server = new RLOGServer();
        RLOGWriter writer = new RLOGWriter("/sdcard/FIRST/", "logs.rlog");
        server.start();
        writer.start();
        Logger.addDataReceiver(server);
        Logger.addDataReceiver(writer);

        Logger.recordMetadata("OpMode/nonsense", "something blabla");
        Logger.recordMetadata("limelight/name",limelight.getDeviceName());

        Logger.start();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        double beforeUserStart = Logger.getTimestamp();
        Logger.periodicBeforeUser();
        double beforeUserEnd = Logger.getTimestamp();

        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // logging pose
        Pose2d pose2d = new Pose2d(follower.getPose().getX(), follower.getPose().getY(), new Rotation2d(follower.getPose().getHeading()));
        Logger.recordOutput("OpMode/Pose2D", pose2d);

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles orientation = result.getBotpose().getOrientation();

            Logger.recordOutput("limelight/Pose2D", new Pose2d(botpose.x, botpose.y, new Rotation2d(orientation.getYaw(AngleUnit.RADIANS))));

            telemetry.addData("Pose x", botpose.x);
            telemetry.addData("Pose y", botpose.y);
            telemetry.addData("Pose heading ", orientation.getYaw(AngleUnit.DEGREES));
        }

        LLStatus llStatus = limelight.getStatus();
        Logger.recordOutput("limelight/FPS", llStatus.getFps());
        Logger.recordOutput("limelight/Cpu", llStatus.getCpu());
        Logger.recordOutput("limelight/Ram", llStatus.getRam());

        //logging the loop
        double afterUserStart = Logger.getTimestamp();
        Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
        );
    }

    @Override
    public void stop() {
        Logger.end();

    }
}
