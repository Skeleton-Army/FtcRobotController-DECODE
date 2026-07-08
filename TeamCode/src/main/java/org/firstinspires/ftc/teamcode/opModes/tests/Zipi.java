package org.firstinspires.ftc.teamcode.opModes.tests;

import android.graphics.Path;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commands.GoToArtifactCommand;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.opModes.TeleOpApp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.Artifact;

import java.io.File;
import java.util.Random;
import java.util.function.LongBinaryOperator;

@TeleOp
public class Zipi extends OpMode {
    Vision vision;
    Follower follower;
    GamepadEx gamepadEx;

    @Override
    public void init() {
        // i think post stop listener is called after log is stopped
        //safe for context to be null, as its unused in the implementation
        //String logFile = RobotLog.getMatchLogFilename(null, OpModeManager.getActiveOpModeName(), 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(TeleOpApp.X_OFFSET, TeleOpApp.Y_OFFSET));
        gamepadEx = new GamepadEx(gamepad1);
        vision = new Vision(hardwareMap, follower.poseTracker, VisionConfig.DETECTION_PIPELINE);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();

        CommandScheduler.getInstance().schedule(
                new GoToArtifactCommand(follower, vision, Alliance.RED)
        );
        telemetry.update();
    }


}
