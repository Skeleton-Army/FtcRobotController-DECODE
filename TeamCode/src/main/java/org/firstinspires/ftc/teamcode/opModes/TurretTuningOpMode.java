package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * TurretTuningOpMode
 *
 * Full system identification for the turret using the Shooter class for all
 * hardware access — same pattern as TurretTuner.  Raw turret power is sent
 * via Shooter.setTurretRawPower().  The flywheel during Kgyro is managed via
 * Shooter.setRPM() + Shooter.periodicFlywheelOnly() so the full flywheel PID
 * runs without the turret PID fighting the sweep controller.
 *
 * ── Phase order ──────────────────────────────────────────────────────────────
 *
 *   Phase 0 — KS
 *     KS must come first because the PRBS amplitude is set to
 *     max(KsCW, KsCCW) + margin to guarantee every pulse moves the turret.
 *     If PRBS runs below stiction the ARX fit absorbs the dead-zone as a
 *     nonlinearity and all derived gains are wrong.
 *
 *     KS varies with angle (gravity and gear-mesh load change), so we measure
 *     at KS_TEST_ANGLES_RAD and take the worst case.  Settling between
 *     measurements is done by watching encoder velocity, not sleeping a fixed
 *     time.
 *
 *   Phase 1 — PRBS + RLS
 *     Pseudo-random binary excitation + Recursive Least Squares.
 *     Kp, Ki, Kd, Kv, Ka all fall out analytically from the converged theta.
 *
 *   Phase 2 — Step validation  (optional, visual only, no values changed)
 *
 *   Phase 3 — Kgyro
 *     Flywheel spins up.  Constant-velocity sweeps with FW off then on.
 *     Closed-loop voltage residual → Kgyro.
 *     All turret params are locked before this phase.
 *
 *   Phase 4 — Save → turret_tuned.json
 *
 * ── Voltage clamping ─────────────────────────────────────────────────────────
 *
 *   u is the desired motor voltage in Volts.  Motor power = u / vBatt.
 *   During tuning all control loops clamp to ±TUNING_SAFETY_VOLTS (9 V) so a
 *   hard-stop impact does not damage the mechanism.  Competition code in
 *   Shooter.updateTurretPID() should use ±12 V (full battery range).
 *
 * ── Loop timing ──────────────────────────────────────────────────────────────
 *
 *   preciseSleep() sleeps for the bulk of the remaining 40 ms period (which
 *   releases the thread) then busy-waits only the final 1.5 ms for accuracy.
 *   This gives ~1 ms jitter without burning 100% CPU for 40 ms straight.
 *   Actual dt is tracked per-loop and fed into the control laws so gains are
 *   correct even when a loop takes slightly longer than 40 ms.
 *
 * ── Required additions to Shooter.java (see Shooter_additions.java) ──────────
 *   setTurretRawPower(double normalizedPower)
 *   periodicFlywheelOnly()
 *   loadTunedParams() + parseJsonField()
 *   5 instance variables (turretKv, turretKa, turretKsCw, turretKsCcw, turretKgyro)
 *   Updated updateTurretPID() tail block
 */
@TeleOp(name = "Turret System ID", group = "Tuning")
public class TurretTuningOpMode extends LinearOpMode {

    // =========================================================================
    //  Constants
    // =========================================================================

    // ── Timing ────────────────────────────────────────────────────────────────
    private static final double T_SEC        = 0.04;          // nominal loop period (s)
    private static final long   T_NS         = 40_000_000L;   // nominal loop period (ns)
    private static final long   BUSY_TAIL_NS = 1_500_000L;    // busy-wait tail for precision

    // ── Voltage limits ────────────────────────────────────────────────────────
    /**
     * Maximum command voltage during all tuning phases.
     * Motor power = u / vBatt, so ±9 V / 12 V = 75% max power during ID.
     * Competition code should use ±12 V for full range.
     */
    private static final double TUNING_SAFETY_VOLTS = 9.0;

    // ── RLS ───────────────────────────────────────────────────────────────────
    private static final double LAMBDA           = 0.97;   // forgetting factor
    private static final int    PRBS_TICKS       = 5;      // PRBS held for 5 × 40ms = 200ms
    private static final int    PRBS_SAMPLES     = 750;    // 30 s of data
    private static final double PRBS_KS_MARGIN_V = 0.5;   // extra V above max(KS)

    // ── Desired closed-loop poles ─────────────────────────────────────────────
    private static final double TARGET_BW_HZ = 4.5;
    private static final double TARGET_ZETA  = 0.85;

    // ── KS calibration ────────────────────────────────────────────────────────
    private static final double KS_RAMP_RATE     = 0.03;   // V per loop ≈ 0.75 V/s
    private static final double KS_MAX_VOLTAGE   = 4.0;    // V ceiling
    private static final double KS_MOTION_THRESH = 0.006;  // rad/s — "just started moving"
    private static final double KS_MARGIN        = 1.05;   // 5% headroom over measured

    /**
     * Angles at which KS is measured (radians).
     * Gravity and gear-mesh loading vary with turret angle; we measure at
     * multiple poses and take the worst case so competition always has enough
     * feedforward to break stiction.
     */
    private static final double[] KS_TEST_ANGLES_RAD = {
            Math.toRadians(-30),
            Math.toRadians(0),
            Math.toRadians(30)
    };

    /**
     * Conservative Kp (V/rad) for positioning during the KS phase only —
     * before the ARX model exists.  Not used anywhere else.
     */
    private static final double KS_POSITION_KP = 15.0;

    // ── Kgyro calibration ─────────────────────────────────────────────────────
    /** Turret output-shaft velocity used for constant-velocity sweeps (rad/s). */
    private static final double KGYRO_TURRET_VEL_RPS = 0.35;
    /** Total arc swept per direction. */
    private static final double KGYRO_RANGE_RAD      = Math.toRadians(40);
    /**
     * Flywheel RPM for the Kgyro measurement.
     * Higher RPM → larger gyro torque → better signal-to-noise.
     * Set to a value the flywheel can reach in KGYRO_FW_SETTLE_S seconds.
     */
    private static final double KGYRO_FW_RPM      = 2500;
    private static final double KGYRO_FW_SETTLE_S = 3.5;
    /** Discard this fraction of each sweep as acceleration transient. */
    private static final double KGYRO_TRANSIENT_FRAC = 0.25;

    // =========================================================================
    //  Hardware / subsystems
    // =========================================================================
    private Shooter       shooter;
    private Follower      follower;
    private VoltageSensor voltageSensor;

    // =========================================================================
    //  RLS state
    //  ARX model:  y[k] = a1·y[k-1] + a2·y[k-2] + b1·u[k-1] + b2·u[k-2]
    // =========================================================================
    private final double[]   theta = {1.5, -0.5, 0.001, 0.0005};
    private final double[][] P = {
            {100, 0, 0, 0}, {0, 100, 0, 0}, {0, 0, 100, 0}, {0, 0, 0, 100}
    };
    private final double[] yBuf = new double[3]; // position history (rad)
    private final double[] uBuf = new double[3]; // voltage history  (V)

    private int    prbsReg     = 0b1010111;
    private int    prbsCounter = 0;
    private double prbsOutput  = 0;

    // =========================================================================
    //  Results
    // =========================================================================
    private double savedKp    = 0, savedKi    = 0, savedKd    = 0;
    private double savedKv    = 0, savedKa    = 0;
    private double savedKsCw  = 0, savedKsCcw = 0;
    private double savedKgyro = 0;
    private double predRms    = 0;

    // =========================================================================
    //  runOpMode
    // =========================================================================
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        initSubsystems();

        telemetry.addLine("=== TURRET FULL SYSTEM ID ===");
        telemetry.addLine("Phase order:  KS → PRBS → [validate] → Kgyro → save");
        telemetry.addLine("Flywheel stays OFF until the Kgyro phase.");
        telemetry.addLine("Keep turret clear of hard stops.  Press START.");
        telemetry.update();
        waitForStart();

        // ── Phase 0: KS ──────────────────────────────────────────────────────
        telemetry.addLine("Phase 0: KS calibration...");
        telemetry.update();
        runPhaseKS();
        if (!opModeIsActive()) return;
        shooter.setTurretRawPower(0);

        // ── Phase 1: PRBS + RLS ───────────────────────────────────────────────
        double prbsAmplitude = Math.max(savedKsCw, savedKsCcw) + PRBS_KS_MARGIN_V;
        waitForPrompt(
                "KS done.  CW=%.4f V   CCW=%.4f V\n" +
                        "PRBS amplitude → %.2f V  (KS_max + %.1f V)\n" +
                        "(A) run PRBS identification (~30 s)   (B) abort",
                savedKsCw, savedKsCcw, prbsAmplitude, PRBS_KS_MARGIN_V);
        if (!gamepad1.a || !opModeIsActive()) return;

        runPhasePRBS();
        if (!opModeIsActive()) return;
        shooter.setTurretRawPower(0);

        // ── Phase 2: Step validation ──────────────────────────────────────────
        waitForPrompt(
                "PRBS done.\n" +
                        "Kp=%.3f  Ki=%.4f  Kd=%.4f\n" +
                        "Kv=%.5f  Ka=%.5f  PredRMS=%.5f\n" +
                        "(A) step validation   (B) skip",
                savedKp, savedKi, savedKd, savedKv, savedKa, predRms);
        if (gamepad1.a) {
            runPhaseStepValidation();
            if (!opModeIsActive()) return;
        }
        shooter.setTurretRawPower(0);

        // ── Phase 3: Kgyro ────────────────────────────────────────────────────
        waitForPrompt(
                "All turret params locked.\n" +
                        "Next phase spins flywheel to ~%.0f RPM.\n" +
                        "Robot must be on the ground and secured.\n" +
                        "(A) spin flywheel + tune Kgyro   (B) skip → Kgyro = 0",
                KGYRO_FW_RPM);
        if (gamepad1.a) {
            runPhaseKgyro();
            if (!opModeIsActive()) return;
        } else {
            savedKgyro = 0;
            telemetry.addLine("Kgyro skipped — gyro feedforward disabled.");
            telemetry.update();
            sleep(600);
        }

        shooter.setTurretRawPower(0);
        shooter.setRPM(0);

        // ── Phase 4: Save ─────────────────────────────────────────────────────
        saveAllParams();

        telemetry.addLine("=== SAVED → turret_tuned.json ===");
        telemetry.addData("Kp / Ki / Kd",  "%.4f / %.4f / %.4f", savedKp,   savedKi,   savedKd);
        telemetry.addData("Kv / Ka",        "%.5f / %.5f",        savedKv,   savedKa);
        telemetry.addData("KsCW / KsCCW",  "%.4f / %.4f",         savedKsCw, savedKsCcw);
        telemetry.addData("Kgyro",          "%.7f",               savedKgyro);
        telemetry.addData("Pred RMS (rad)", "%.5f",               predRms);
        telemetry.update();

        while (opModeIsActive()) idle();
    }

    // =========================================================================
    //  Phase 0 — KS  (static / coulomb friction)
    // =========================================================================
    private void runPhaseKS() {
        double maxCW = 0, maxCCW = 0;

        for (double angleRad : KS_TEST_ANGLES_RAD) {
            if (!opModeIsActive()) return;

            // Move to test angle with conservative Kp before ARX model exists
            moveToAngle(angleRad, KS_POSITION_KP, 1.5);
            // Wait for encoder velocity to actually reach zero — no fixed sleep
            waitForVelocitySettle(0.008, 1500);
            if (!opModeIsActive()) return;

            double ksCW = rampUntilMotion(+1.0,
                    String.format("KS CW  @ %.0f°", Math.toDegrees(angleRad)));
            if (!opModeIsActive()) return;

            // Return to angle and re-settle before CCW measurement
            moveToAngle(angleRad, KS_POSITION_KP, 1.0);
            waitForVelocitySettle(0.008, 1000);
            if (!opModeIsActive()) return;

            double ksCCW = rampUntilMotion(-1.0,
                    String.format("KS CCW @ %.0f°", Math.toDegrees(angleRad)));
            if (!opModeIsActive()) return;

            maxCW  = Math.max(maxCW,  ksCW);
            maxCCW = Math.max(maxCCW, ksCCW);

            telemetry.addData("Tested angle (°)",   "%.0f", Math.toDegrees(angleRad));
            telemetry.addData("KsCW  worst so far",  "%.4f V  (this: %.4f)", maxCW,  ksCW);
            telemetry.addData("KsCCW worst so far",  "%.4f V  (this: %.4f)", maxCCW, ksCCW);
            telemetry.update();
        }

        moveToAngle(0, KS_POSITION_KP, 1.5);

        // 5% headroom so we reliably break stiction at competition voltage
        savedKsCw  = maxCW  * KS_MARGIN;
        savedKsCcw = maxCCW * KS_MARGIN;

        telemetry.addData("Phase",       "KS done");
        telemetry.addData("KsCW  (V)",  "%.4f  (worst case + %.0f%% margin)", savedKsCw,  (KS_MARGIN - 1) * 100);
        telemetry.addData("KsCCW (V)",  "%.4f  (worst case + %.0f%% margin)", savedKsCcw, (KS_MARGIN - 1) * 100);
        telemetry.update();
        sleep(600);
    }

    /**
     * Ramps open-loop voltage in the given direction until the turret velocity
     * exceeds KS_MOTION_THRESH.  Returns the threshold voltage in Volts.
     */
    private double rampUntilMotion(double dir, String label) {
        double vRamp    = 0;
        double measured = 0;

        while (opModeIsActive() && vRamp < KS_MAX_VOLTAGE) {
            long t0 = System.nanoTime();
            vRamp += KS_RAMP_RATE;

            double vBatt = voltageSensor.getVoltage();
            shooter.setTurretRawPower(dir * vRamp / vBatt);

            double vel = Math.abs(shooter.getTurretAngleVel());
            telemetry.addData("Phase",       label);
            telemetry.addData("Voltage",     "%.3f V", vRamp * dir);
            telemetry.addData("Vel (rad/s)", "%.4f", vel);
            telemetry.update();

            if (vel > KS_MOTION_THRESH) {
                measured = vRamp;
                break;
            }
            preciseSleep(t0);
        }

        shooter.setTurretRawPower(0);
        return measured;
    }

    // =========================================================================
    //  Phase 1 — PRBS + RLS
    // =========================================================================
    private void runPhasePRBS() {
        double prbsAmplitude = Math.max(savedKsCw, savedKsCcw) + PRBS_KS_MARGIN_V;

        prbsReg     = 0b1010111;
        prbsCounter = 0;
        prbsOutput  = prbsAmplitude;

        int    n      = 0;
        double rmsAcc = 0;

        while (opModeIsActive() && n < PRBS_SAMPLES) {
            long t0 = System.nanoTime();

            double pos  = shooter.getTurretAngle(AngleUnit.RADIANS);
            yBuf[2] = yBuf[1]; yBuf[1] = yBuf[0]; yBuf[0] = pos;

            if (++prbsCounter >= PRBS_TICKS) {
                prbsCounter = 0;
                int fb  = ((prbsReg >> 6) ^ (prbsReg >> 5)) & 1;
                prbsReg = ((prbsReg << 1) | fb) & 0x7F;
                prbsOutput = ((prbsReg & 1) == 1) ? prbsAmplitude : -prbsAmplitude;
            }

            double u = prbsOutput;
            if (pos >  TURRET_MAX - 0.12) u = -Math.abs(u);
            if (pos <  TURRET_MIN + 0.12) u =  Math.abs(u);

            double vBatt = voltageSensor.getVoltage();
            shooter.setTurretRawPower(u / vBatt);

            uBuf[2] = uBuf[1]; uBuf[1] = uBuf[0]; uBuf[0] = u;

            if (n >= 2) {
                double[] phi = {yBuf[1], yBuf[2], uBuf[1], uBuf[2]};
                double yHat  = dot(phi, theta);
                double err   = yBuf[0] - yHat;
                rmsAcc      += err * err;

                double[] Pphi = matVecMul(P, phi);
                double denom  = LAMBDA + dot(phi, Pphi);
                double[] K    = scale(Pphi, 1.0 / denom);
                addInPlace(theta, scale(K, err));
                updateP(P, K, phi);
            }
            n++;

            telemetry.addData("Phase",     "PRBS + RLS  [FW OFF]");
            telemetry.addData("Amplitude", "%.2f V  (KS_max + %.1f)", prbsAmplitude, PRBS_KS_MARGIN_V);
            telemetry.addData("Progress",  "%d / %d  (%.0f%%)", n, PRBS_SAMPLES, 100.0 * n / PRBS_SAMPLES);
            telemetry.addData("a1 / a2",   "%.4f / %.4f", theta[0], theta[1]);
            telemetry.addData("b1 / b2",   "%.5f / %.5f", theta[2], theta[3]);
            if (n > 2) {
                double pe = yBuf[0] - dot(
                        new double[]{yBuf[1], yBuf[2], uBuf[1], uBuf[2]}, theta);
                telemetry.addData("Pred err (rad)", "%.5f", pe);
            }
            telemetry.update();

            preciseSleep(t0);
        }

        predRms = Math.sqrt(rmsAcc / Math.max(n - 2, 1));

        double[] pid = computePIDFromARX(theta);
        savedKp = pid[0];
        savedKi = pid[1];
        savedKd = pid[2];
        savedKv = computeKvFromARX(theta);
        savedKa = computeKaFromARX(theta);
    }

    // =========================================================================
    //  Phase 2 — Step validation  (optional, flywheel OFF, no values changed)
    // =========================================================================
    private void runPhaseStepValidation() {
        double[] targets = {
                Math.toRadians(30), 0.0,
                Math.toRadians(-25), 0.0
        };
        double integrator = 0, prevError = 0;
        long   prevTime   = System.nanoTime();

        for (double target : targets) {
            long stepStart = System.nanoTime();
            while (opModeIsActive()
                    && (System.nanoTime() - stepStart) < 3_000_000_000L) {
                long   t0  = System.nanoTime();
                double dt  = (t0 - prevTime) / 1e9;
                prevTime   = t0;
                if (dt < 0.001) dt = T_SEC;

                double pos = shooter.getTurretAngle(AngleUnit.RADIANS);
                double e   = target - pos;

                integrator = clamp(integrator + e * dt, -2.0, 2.0);
                double eDot = (e - prevError) / dt;
                prevError   = e;

                double ks = e > 0 ? savedKsCcw : savedKsCw;
                double u  = savedKp * e
                        + savedKi * integrator
                        + savedKd * eDot
                        + Math.signum(e) * ks;
                u = clamp(u, -TUNING_SAFETY_VOLTS, TUNING_SAFETY_VOLTS);

                shooter.setTurretRawPower(u / voltageSensor.getVoltage());

                telemetry.addData("Phase",      "Step validation  [FW OFF]");
                telemetry.addData("Target (°)", "%.1f", Math.toDegrees(target));
                telemetry.addData("Pos    (°)", "%.1f", Math.toDegrees(pos));
                telemetry.addData("Error  (°)", "%.2f", Math.toDegrees(e));
                telemetry.update();

                preciseSleep(t0);
            }
        }
        shooter.setTurretRawPower(0);
    }

    // =========================================================================
    //  Phase 3 — Kgyro  (flywheel ON, all turret params locked)
    // =========================================================================
    private void runPhaseKgyro() {

        // ── A. Baseline sweeps — flywheel OFF ─────────────────────────────────
        shooter.setRPM(0);

        telemetry.addData("Phase", "Kgyro — baseline CW  [FW OFF]");
        telemetry.update();
        double uBaseCW = sweepConstantVelocity(+KGYRO_TURRET_VEL_RPS, KGYRO_RANGE_RAD, false);
        if (!opModeIsActive()) return;

        telemetry.addData("Phase", "Kgyro — baseline CCW  [FW OFF]");
        telemetry.update();
        double uBaseCCW = sweepConstantVelocity(-KGYRO_TURRET_VEL_RPS, KGYRO_RANGE_RAD, false);
        if (!opModeIsActive()) return;

        // ── B. Spin flywheel and wait for steady RPM ──────────────────────────
        shooter.setRPM(KGYRO_FW_RPM);

        long spinStart = System.nanoTime();
        while (opModeIsActive()
                && (System.nanoTime() - spinStart) < (long)(KGYRO_FW_SETTLE_S * 1e9)) {
            shooter.periodicFlywheelOnly(); // keeps flywheel PID alive
            telemetry.addData("Phase",  "Kgyro — FW settling");
            telemetry.addData("FW RPM", "%.0f  /  %.0f target", shooter.getRPM(), KGYRO_FW_RPM);
            telemetry.update();
            sleep(50);
        }

        double omegaFW = shooter.getRPM() * 2.0 * Math.PI / 60.0; // rad/s

        // ── C. Gyro sweeps — flywheel ON ──────────────────────────────────────
        telemetry.addData("Phase", "Kgyro — gyro CW  [FW ON]");
        telemetry.update();
        double uGyroCW = sweepConstantVelocity(+KGYRO_TURRET_VEL_RPS, KGYRO_RANGE_RAD, true);
        if (!opModeIsActive()) return;

        telemetry.addData("Phase", "Kgyro — gyro CCW  [FW ON]");
        telemetry.update();
        double uGyroCCW = sweepConstantVelocity(-KGYRO_TURRET_VEL_RPS, KGYRO_RANGE_RAD, true);
        if (!opModeIsActive()) return;

        shooter.setRPM(0);

        // ── D. Fit Kgyro ──────────────────────────────────────────────────────
        // Gyro disturbance is anti-symmetric in turret direction:
        //   CW  sweep: ΔV_cw  = u_gyro_cw  − u_base_cw   (positive if gyro opposes CW)
        //   CCW sweep: ΔV_ccw = u_base_ccw − u_gyro_ccw  (sign flip for same convention)
        // Averaging both cancels residual static asymmetry not removed by KS.
        double deltaVcw  = uGyroCW  - uBaseCW;
        double deltaVccw = uBaseCCW - uGyroCCW;
        double deltaV    = (deltaVcw + deltaVccw) / 2.0;

        if (omegaFW < 50.0 || Math.abs(deltaV) < 0.005) {
            savedKgyro = 0;
            telemetry.addLine("WARNING: Kgyro ≈ 0 — flywheel may not have reached speed.");
        } else {
            savedKgyro = deltaV / (omegaFW * KGYRO_TURRET_VEL_RPS);
            savedKgyro = clamp(savedKgyro, 0.0, 0.015);
        }

        telemetry.addData("Phase",          "Kgyro done");
        telemetry.addData("ω_fw (r/s)",     "%.1f", omegaFW);
        telemetry.addData("uBase CW / CCW", "%.4f / %.4f", uBaseCW,  uBaseCCW);
        telemetry.addData("uGyro CW / CCW", "%.4f / %.4f", uGyroCW,  uGyroCCW);
        telemetry.addData("ΔV CW / CCW",    "%.4f / %.4f", deltaVcw, deltaVccw);
        telemetry.addData("Kgyro",          "%.7f", savedKgyro);
        telemetry.update();
        sleep(1000);
    }

    /**
     * Drives the turret at constant velocity using the full locked FF+PID.
     * During FW-on sweeps the flywheel PID is kept alive via periodicFlywheelOnly().
     *
     * Returns mean |u| in Volts during the steady-state window
     * (first KGYRO_TRANSIENT_FRAC discarded).
     *
     * Voltage is clamped to ±TUNING_SAFETY_VOLTS — not ±12 V — because this
     * is a measurement run, not competition.
     */
    private double sweepConstantVelocity(double velRps, double rangeRad, boolean fwOn) {
        double startPos  = shooter.getTurretAngle(AngleUnit.RADIANS);
        double targetPos = startPos + Math.signum(velRps) * rangeRad;
        targetPos = clamp(targetPos, TURRET_MIN + 0.06, TURRET_MAX - 0.06);

        double actualRange  = targetPos - startPos;
        double durationS    = Math.abs(actualRange / velRps);
        long   durationNs   = (long)(durationS * 1e9);
        long   runStart     = System.nanoTime();
        long   measureStart = runStart + (long)(durationNs * KGYRO_TRANSIENT_FRAC);
        long   prevTime     = runStart;

        double rampPos    = startPos;
        double integrator = 0, prevError = 0;
        double voltSum    = 0;
        int    voltCount  = 0;

        while (opModeIsActive()
                && (System.nanoTime() - runStart) < durationNs) {
            long   t0  = System.nanoTime();
            double dt  = (t0 - prevTime) / 1e9;
            prevTime   = t0;
            if (dt < 0.001) dt = T_SEC;

            rampPos += velRps * dt;
            rampPos  = clamp(rampPos, TURRET_MIN + 0.06, TURRET_MAX - 0.06);

            double pos   = shooter.getTurretAngle(AngleUnit.RADIANS);
            double error = rampPos - pos;

            integrator = clamp(integrator + error * dt, -1.5, 1.5);
            double eDot = (error - prevError) / dt;
            prevError   = error;

            // Full FF+PID — identical to what Shooter.java runs at competition
            double ks = velRps > 0 ? savedKsCcw : savedKsCw;
            double u  = savedKp * error
                    + savedKi * integrator
                    + savedKd * eDot
                    + savedKv * velRps     // velocity FF
                    // Ka × 0 accel = 0 at constant speed
                    + Math.signum(velRps) * ks;
            u = clamp(u, -TUNING_SAFETY_VOLTS, TUNING_SAFETY_VOLTS);

            shooter.setTurretRawPower(u / voltageSensor.getVoltage());

            // Keep flywheel PID alive during FW-on sweeps
            if (fwOn) {
                shooter.setRPM(KGYRO_FW_RPM);
                shooter.periodicFlywheelOnly();
            }

            if (t0 >= measureStart) {
                voltSum  += Math.abs(u);
                voltCount++;
            }

            String dir = (velRps > 0 ? "CW" : "CCW") + (fwOn ? " [FW ON]" : " [FW OFF]");
            telemetry.addData("Sweep",     dir);
            telemetry.addData("Pos (°)",   "%.1f → %.1f",
                    Math.toDegrees(pos), Math.toDegrees(targetPos));
            telemetry.addData("Vel (r/s)", "%.3f  /  %.3f target",
                    shooter.getTurretAngleVel(), velRps);
            telemetry.addData("u (V)",     "%.3f", u);
            if (fwOn) telemetry.addData("FW RPM", "%.0f  /  %.0f", shooter.getRPM(), KGYRO_FW_RPM);
            telemetry.update();

            preciseSleep(t0);
        }

        shooter.setTurretRawPower(0);
        moveToAngle(startPos, savedKp, 1.2);
        return voltCount > 0 ? voltSum / voltCount : 0.0;
    }

    // =========================================================================
    //  Save
    // =========================================================================
    private void saveAllParams() {
        String json = String.format(
                "{\"a1\":%.7f,\"a2\":%.7f,\"b1\":%.7f,\"b2\":%.7f," +
                        "\"Kp\":%.5f,\"Ki\":%.5f,\"Kd\":%.5f," +
                        "\"Kv\":%.6f,\"Ka\":%.6f," +
                        "\"Ks_cw\":%.5f,\"Ks_ccw\":%.5f," +
                        "\"Kgyro\":%.8f," +
                        "\"rms\":%.7f,\"timestamp\":%d}",
                theta[0], theta[1], theta[2], theta[3],
                savedKp, savedKi, savedKd,
                savedKv, savedKa,
                savedKsCw, savedKsCcw,
                savedKgyro,
                predRms,
                System.currentTimeMillis());

        ReadWriteFile.writeFile(
                AppUtil.getInstance().getSettingsFile("turret_tuned.json"), json);
    }

    // =========================================================================
    //  Analytical gain computation from ARX  theta = [a1, a2, b1, b2]
    // =========================================================================

    /** Pole-placement PID from desired (ζ, ωn) mapped to z via z = e^(sT). */
    private double[] computePIDFromARX(double[] th) {
        double wn    = 2.0 * Math.PI * TARGET_BW_HZ;
        double sigma = TARGET_ZETA * wn;
        double wd    = wn * Math.sqrt(1.0 - TARGET_ZETA * TARGET_ZETA);
        double r     = Math.exp(-sigma * T_SEC);
        double zRe   = r * Math.cos(wd * T_SEC);

        double b1 = th[2];
        if (Math.abs(b1) < 1e-9) b1 = 1e-9;

        double Kp = clamp((2.0 * zRe - th[0]) / b1, 10.0, 300.0);
        double Ki = Kp * T_SEC * wn / 8.0;
        double Kd = Kp / (wn * 2.5 / T_SEC);
        return new double[]{Kp, Ki, Kd};
    }

    /**
     * Kv — velocity feedforward.
     * At steady-state constant velocity:  Kv = T × (1 − a1 − a2) / (b1 + b2)
     * Units: V / (rad/s output shaft)
     */
    private double computeKvFromARX(double[] th) {
        double num = T_SEC * (1.0 - th[0] - th[1]);
        double den = th[2] + th[3];
        if (Math.abs(den) < 1e-9) return TURRET_KV;
        return clamp(num / den, 0.01, 5.0);
    }

    /**
     * Ka — acceleration feedforward.
     * Motor time constant from second discrete pole:  τ_m = −T / ln(−a2)
     * Ka = τ_m × Kv
     * Units: V / (rad/s² output shaft)
     */
    private double computeKaFromARX(double[] th) {
        double a2 = th[1];
        if (a2 >= 0.0 || a2 <= -1.0) return TURRET_KA;
        double tauM = -T_SEC / Math.log(-a2);
        return clamp(tauM * computeKvFromARX(th), 0.0001, 0.5);
    }

    // =========================================================================
    //  Utilities
    // =========================================================================

    private void initSubsystems() {
        follower = Constants.createFollower(hardwareMap);
        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, Alliance.RED);

        // We control turret and flywheel directly in this OpMode
        shooter.setHorizontalManualMode(true);
        shooter.setVerticalManualMode(true);
        // Flywheel is driven explicitly via setRPM + periodicFlywheelOnly
        shooter.setUpdateFlywheel(false);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * Moves the turret to {@code targetRad} using Kp-only control for up to
     * {@code timeoutS} seconds.  Exits early when within 1.5°.
     */
    private void moveToAngle(double targetRad, double kp, double timeoutS) {
        long end = System.nanoTime() + (long)(timeoutS * 1e9);
        while (opModeIsActive() && System.nanoTime() < end) {
            long   t0 = System.nanoTime();
            double e  = targetRad - shooter.getTurretAngle(AngleUnit.RADIANS);
            if (Math.abs(e) < Math.toRadians(1.5)) break;
            double u = clamp(kp * e, -TUNING_SAFETY_VOLTS, TUNING_SAFETY_VOLTS);
            shooter.setTurretRawPower(u / voltageSensor.getVoltage());
            preciseSleep(t0);
        }
        shooter.setTurretRawPower(0);
    }

    /**
     * Waits until turret velocity falls below {@code threshRads} rad/s, or
     * until {@code timeoutMs} elapses.  Polls every 20 ms — no fixed sleep.
     */
    private void waitForVelocitySettle(double threshRads, long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            if (Math.abs(shooter.getTurretAngleVel()) < threshRads) break;
            sleep(20);
        }
    }

    /**
     * Blocks until gamepad1 A or B is pressed.
     * Debounces any currently-held button before waiting for a fresh press.
     */
    private void waitForPrompt(String fmt, Object... args) {
        shooter.setTurretRawPower(0);
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) idle();
        telemetry.addData("──", String.format(fmt, args));
        telemetry.addLine("(A) = proceed   (B) = skip / abort");
        telemetry.update();
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b) idle();
    }

    /**
     * Hybrid precise sleep.
     *
     * Releases the thread for the bulk of the remaining 40 ms period, then
     * busy-waits only the final BUSY_TAIL_NS (1.5 ms) for timing accuracy.
     *
     * Result: ~1 ms jitter without burning 100% CPU for the full period.
     * Pure busy-wait has ~0.05 ms jitter but wastes the CPU.
     * Pure sleep() has ~5–10 ms jitter on Android.
     */
    private void preciseSleep(long startNs) {
        long targetNs  = startNs + T_NS;
        long remaining = targetNs - System.nanoTime();
        long sleepMs   = (remaining - BUSY_TAIL_NS) / 1_000_000L;
        if (sleepMs > 0) sleep(sleepMs);
        while (opModeIsActive() && System.nanoTime() < targetNs) { /* spin */ }
    }

    // =========================================================================
    //  RLS linear algebra
    // =========================================================================

    private double dot(double[] a, double[] b) {
        double s = 0;
        for (int i = 0; i < a.length; i++) s += a[i] * b[i];
        return s;
    }

    private double[] scale(double[] v, double s) {
        double[] r = new double[v.length];
        for (int i = 0; i < v.length; i++) r[i] = v[i] * s;
        return r;
    }

    private void addInPlace(double[] a, double[] b) {
        for (int i = 0; i < a.length; i++) a[i] += b[i];
    }

    private double[] matVecMul(double[][] M, double[] v) {
        int n = v.length;
        double[] r = new double[n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) r[i] += M[i][j] * v[j];
        return r;
    }

    /** In-place covariance update:  P ← (P − K·φᵀ) / λ */
    private void updateP(double[][] P, double[] K, double[] phi) {
        int n = K.length;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                P[i][j] = (P[i][j] - K[i] * phi[j]) / LAMBDA;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}