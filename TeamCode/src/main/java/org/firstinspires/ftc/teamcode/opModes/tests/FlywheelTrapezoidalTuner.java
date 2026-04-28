package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Flywheel Trapezoidal Ramp Tuner", group = "Tuning")
public class FlywheelTrapezoidalTuner extends LinearOpMode {

    // ─── TUNE THESE ──────────────────────────────────────────────────
    private static final double START_RPM      = 0;     // RPM at start
    private static final double CRUISE_RPM     = 2500;  // RPM to cruise at
    private static final double END_RPM        = 0;     // RPM at end (set >0 to land somewhere other than 0)

    private static final double ACCEL_DURATION = 3.0;   // seconds to ramp up
    private static final double CRUISE_DURATION = 8.0;  // seconds to hold at CRUISE_RPM
    private static final double DECEL_DURATION = 3.0;   // seconds to ramp down
    // ─────────────────────────────────────────────────────────────────

    private double lastTargetRPM   = 0;
    private double lastMeasuredRPM = 0;
    private double lastLoopTime    = -1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IShooterCalculator shooterCalc = new ShooterCalculator(new ShooterCoefficients());
        Follower follower = Constants.createFollower(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, shooterCalc, Alliance.RED);
        shooter.setUpdateFlywheel(false);
        shooter.disableTurret();

        telemetry.addLine("Flywheel Trapezoidal Ramp Tuner");
        telemetry.addLine("Press PLAY to start.");
        telemetry.update();

        waitForStart();

        ElapsedTime phaseTimer = new ElapsedTime();

        // ── Phase 1: Accelerate ───────────────────────────────────────
        phaseTimer.reset();
        while (opModeIsActive() && phaseTimer.seconds() < ACCEL_DURATION) {
            double t = phaseTimer.seconds() / ACCEL_DURATION;
            double targetRPM = START_RPM + t * (CRUISE_RPM - START_RPM);
            runLoop(shooter, targetRPM, "ACCEL", phaseTimer.seconds(), ACCEL_DURATION);
        }

        // ── Phase 2: Cruise ───────────────────────────────────────────
        phaseTimer.reset();
        while (opModeIsActive() && phaseTimer.seconds() < CRUISE_DURATION) {
            runLoop(shooter, CRUISE_RPM, "CRUISE", phaseTimer.seconds(), CRUISE_DURATION);
        }

        // ── Phase 3: Decelerate ───────────────────────────────────────
        phaseTimer.reset();
        while (opModeIsActive() && phaseTimer.seconds() < DECEL_DURATION) {
            double t = phaseTimer.seconds() / DECEL_DURATION;
            double targetRPM = CRUISE_RPM + t * (END_RPM - CRUISE_RPM);
            runLoop(shooter, targetRPM, "DECEL", phaseTimer.seconds(), DECEL_DURATION);
        }

        // ── Done ──────────────────────────────────────────────────────
        shooter.setRPM(END_RPM);
        shooter.periodic();
        telemetry.addLine("Profile complete.");
        telemetry.update();
        sleep(1000);
    }

    private void runLoop(Shooter shooter, double targetRPM, String phase, double elapsed, double duration) {
        shooter.setRPM(targetRPM);
        shooter.periodic();

        double measuredRPM  = shooter.filteredRPM;
        double now          = System.nanoTime() / 1e9;

        double targetAccelRPM   = 0;
        double measuredAccelRPM = 0;

        if (lastLoopTime > 0) {
            double dt = now - lastLoopTime;
            if (dt > 0.001) {
                targetAccelRPM   = (targetRPM  - lastTargetRPM)   / dt;
                measuredAccelRPM = (measuredRPM - lastMeasuredRPM) / dt;
            }
        }

        lastTargetRPM   = targetRPM;
        lastMeasuredRPM = measuredRPM;
        lastLoopTime    = now;

        telemetry.addData("Phase",            phase);
        telemetry.addData("Elapsed (s)",      "%.2f / %.2f", elapsed, duration);
        telemetry.addData("Target RPM",       "%.0f",  targetRPM);
        telemetry.addData("Actual RPM",       "%.0f",  measuredRPM);
        telemetry.addData("Error RPM",        "%.0f",  targetRPM - measuredRPM);
        telemetry.addData("Target Accel",     "%.1f", targetAccelRPM);
        telemetry.addData("Flywheel Accel",   "%.1f", measuredAccelRPM);
        telemetry.update();
    }
}