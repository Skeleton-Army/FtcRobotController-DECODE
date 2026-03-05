package org.firstinspires.ftc.teamcode.opModes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

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

    private Shooter shooter;
    private Follower follower;

    private ElapsedTime timer;
    private double targetAngleDeg = 0;

    private double sumErrorDeg = 0;
    private int loopCount = 0;
    private double maxErrorDeg = 0;

    private boolean stepToggle = false;
    private boolean active = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        IShooterCalculator shooterCalc = new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS);
        shooter = new Shooter(hardwareMap, follower.poseTracker, shooterCalc, Alliance.RED);

        shooter.setHorizontalManualMode(true);
        shooter.setVerticalManualMode(true);
        shooter.setUpdateFlywheel(false);

        timer = new ElapsedTime();
    }

    @Override
    public void run() {
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

        if (ENABLE_SINE) {
            targetAngleDeg = SINE_AMPLITUDE_DEG * Math.sin(2 * Math.PI * SINE_FREQUENCY_HZ * timer.seconds());
        }

        shooter.updatePIDFCoefficients();
        shooter.setRPM(TUNING_FLYWHEEL_RPM);
        shooter.setHorizontalAngle(Math.toRadians(targetAngleDeg));

        double actualAngleDeg = shooter.getTurretAngle(AngleUnit.DEGREES);
        double errorDeg = Math.abs(targetAngleDeg - actualAngleDeg);

        if (active) updateStats(errorDeg);

        telemetry.addData("Target Angle (Deg)", targetAngleDeg);
        telemetry.addData("Actual Angle (Deg)", actualAngleDeg);
        telemetry.addData("Error (Deg)", errorDeg);
        telemetry.addData("Flywheel RPM", shooter.getRPM());
        telemetry.addData("Avg Error (Deg)", sumErrorDeg / Math.max(1, loopCount));
        telemetry.addData("Max Error (Deg)", maxErrorDeg);
        telemetry.update();
    }

    private void updateStats(double error) {
        sumErrorDeg += error;
        loopCount++;
        if (error > maxErrorDeg) maxErrorDeg = error;
    }

    private void resetStats() {
        sumErrorDeg = 0;
        loopCount = 0;
        maxErrorDeg = 0;
    }
}