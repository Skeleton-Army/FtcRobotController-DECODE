package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Encoder Calibration", group = "Calibration")
public class EncoderCalibration extends LinearOpMode {
    private static final double GOBILDA_TICKS_PER_MM = 19.89436789;
    private static final double ACTUAL_DISTANCE_METERS = 3.0;

    private static final double ACTUAL_DISTANCE_MM   = ACTUAL_DISTANCE_METERS * 1000;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        PinpointLocalizer localizer = (PinpointLocalizer) follower.poseTracker.getLocalizer();

        waitForStart();

        double forwardStart = localizer.getForwardMultiplier();
        double strafeStart  = localizer.getLateralMultiplier();

        while (opModeIsActive()) {
            double fwdTicks    = localizer.getForwardMultiplier() - forwardStart;
            double strafeTicks = localizer.getLateralMultiplier() - strafeStart;

            double calcFwdTPMM    = fwdTicks    / ACTUAL_DISTANCE_MM;
            double calcStrafeTPMM = strafeTicks / ACTUAL_DISTANCE_MM;

            telemetry.addLine("════ ENCODER CALIBRATION ════");
            telemetry.addLine("Drive " + ACTUAL_DISTANCE_METERS + "m forward OR strafe — both logged live.");
            telemetry.addLine("");

            telemetry.addLine("── Forward Pod ──");
            telemetry.addData("Ticks",         "%.0f", fwdTicks);
            telemetry.addData("Distance",      "%.1f mm", fwdTicks / GOBILDA_TICKS_PER_MM);
            telemetry.addData("Calc ticks/mm", "%.8f", calcFwdTPMM);
            telemetry.addData("Error",         "%.3f%%", ((calcFwdTPMM - GOBILDA_TICKS_PER_MM) / GOBILDA_TICKS_PER_MM) * 100);
            telemetry.addLine("");

            telemetry.addLine("── Strafe Pod ──");
            telemetry.addData("Ticks",         "%.0f", strafeTicks);
            telemetry.addData("Distance",      "%.1f mm", strafeTicks / GOBILDA_TICKS_PER_MM);
            telemetry.addData("Calc ticks/mm", "%.8f", calcStrafeTPMM);
            telemetry.addData("Error",         "%.3f%%", ((calcStrafeTPMM - GOBILDA_TICKS_PER_MM) / GOBILDA_TICKS_PER_MM) * 100);

            telemetry.update();
        }
    }
}