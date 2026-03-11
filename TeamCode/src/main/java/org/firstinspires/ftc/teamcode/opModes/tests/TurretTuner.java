package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

import java.util.ArrayDeque;

@Config
@TeleOp(name = "Tuning: Turret PIDF", group = "Tuning")
public class TurretTuner extends ComplexOpMode {
    /** How far the turret swings in each direction (Degrees) */
    public static double SINE_AMPLITUDE_DEG = 45.0;

    /**
     * How fast the turret oscillates (Hz).
     * INCREASE: Faster, more aggressive motion.
     * DECREASE: Slower, smoother motion.
     */
    public static double SINE_FREQUENCY_HZ = 0.5;

    /** Toggle for sine wave motion. Use Gamepad Cross to toggle live. */
    public static boolean ENABLE_SINE = false;

    /** Target flywheel speed. */
    public static double TUNING_FLYWHEEL_RPM = 3000;

    /**
     * Number of loops to ignore after motion starts.
     * Prevents the initial transient from polluting max/avg error.
     * INCREASE: Ignore more of the startup spike.
     * DECREASE: Start tracking sooner.
     */
    public static int SETTLE_LOOPS = 30;

    /**
     * Rolling window size for average error (in loops).
     * INCREASE: Smoother average, slower to react to changes.
     * DECREASE: More reactive, noisier average.
     */
    public static int ROLLING_WINDOW = 30;

    private Shooter shooter;
    private Follower follower;

    private ElapsedTime timer;
    private double targetAngleDeg = 0;

    private final ArrayDeque<Double> errorWindow = new ArrayDeque<>();
    private double rollingSum = 0;
    private double maxErrorDeg = 0;
    private int settleCounter = 0;

    private boolean stepToggle = false;
    private boolean active = false;

    private boolean updateFlywheelSolution = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, Alliance.RED);

        shooter.setHorizontalManualMode(true);
        shooter.setVerticalManualMode(true);
        shooter.setUpdateFlywheel(updateFlywheelSolution);

        timer = new ElapsedTime();
    }

    @Override
    public void run() {
        // Force the OpMode to run on 30ms
        ElapsedTime loopTimer = new ElapsedTime();
        double remainingTimeMs = 30.0 - loopTimer.milliseconds();

        if (remainingTimeMs > 0) {
            sleep((long) remainingTimeMs);
        }

        if (gamepad1.crossWasPressed()) {
            ENABLE_SINE = !ENABLE_SINE;
            active = true;
            resetStats();
        }

        if (gamepad1.squareWasPressed()) {
            ENABLE_SINE = false;
            stepToggle = !stepToggle;
            targetAngleDeg = stepToggle ? SINE_AMPLITUDE_DEG : 0;
            active = true;
            resetStats();
        }

        if (gamepad1.triangleWasPressed()) {
            updateFlywheelSolution = !updateFlywheelSolution;
        }

        if (ENABLE_SINE) {
            active = true;
            targetAngleDeg = SINE_AMPLITUDE_DEG * Math.sin(2 * Math.PI * SINE_FREQUENCY_HZ * timer.seconds());
        }

        if (!updateFlywheelSolution) {
            shooter.setUpdateFlywheel(false);
            shooter.setRPM(TUNING_FLYWHEEL_RPM);
        } else {
            shooter.setUpdateFlywheel(true);
        }
        shooter.setHorizontalAngle(Math.toRadians(targetAngleDeg));

        double actualAngleDeg = shooter.getTurretAngle(AngleUnit.DEGREES);
        double errorDeg = Math.abs(targetAngleDeg - actualAngleDeg);

        if (active) {
            if (settleCounter < SETTLE_LOOPS) {
                settleCounter++;
            } else {
                updateStats(errorDeg);
            }
        }

        double rollingAvg = errorWindow.isEmpty() ? 0 : rollingSum / errorWindow.size();

        double[] targetKinematics = shooter.getNetTargetKinematics();
        telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
        telemetry.addLine("----------------------------------");
        telemetry.addData("Target Angle (Deg)", targetAngleDeg);
        telemetry.addData("Actual Angle (Deg)", actualAngleDeg);
        telemetry.addData("Error (Deg)", errorDeg);
        telemetry.addData("Avg Error (Deg) [last " + ROLLING_WINDOW + "]", rollingAvg);
        telemetry.addData("Max Error (Deg)", maxErrorDeg);
        telemetry.addData("Settling", settleCounter < SETTLE_LOOPS ? "YES (" + settleCounter + "/" + SETTLE_LOOPS + ")" : "NO");
        telemetry.addLine("----------------------------------");
        telemetry.addData("Target Velocity (Deg/sec)", targetKinematics[0]);
        telemetry.addData("Actual Velocity (Deg/sec)", shooter.getTurretAngleVel());
        telemetry.addData("Error (Deg/sec)", targetKinematics[0] - shooter.getTurretAngleVel());
        telemetry.addLine("----------------------------------");
        telemetry.addData("Target Acceleration (Deg/sec^2)", targetKinematics[1]);
        telemetry.addData("Actual Acceleration (Deg/sec^2)", shooter.getTurretAngleVel());
        telemetry.addData("Error (Deg/sec^2)", targetKinematics[1] - shooter.getTurretAngleVel());
        telemetry.addLine("----------------------------------");
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.update();
    }

    private void updateStats(double error) {
        if (errorWindow.size() >= ROLLING_WINDOW) {
            rollingSum -= errorWindow.removeFirst();
        }
        errorWindow.addLast(error);
        rollingSum += error;

        if (error > maxErrorDeg) maxErrorDeg = error;
    }

    private void resetStats() {
        errorWindow.clear();
        rollingSum = 0;
        maxErrorDeg = 0;
        settleCounter = 0;
    }
}