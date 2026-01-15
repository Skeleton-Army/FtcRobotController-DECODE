package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "SysId Data Logger", group = "SysId")
public class SysIdTest extends OpMode {

    /* ================= CONFIG ================= */
    private static final String MOTOR_NAME = "flywheel1";
    private static final String MOTOR_NAME2 = "flywheel2";

    private static final double RAMP_RATE = 0.25; // Volts per second
    private static final double MAX_VOLTAGE = 12.0; // Safety cap
    private static final double STEP_VOLTAGE = 6.0; // For Dynamic test
    private static final double STEP_DELAY = 1.0;   // Seconds before step applied

    /* ================= STATE MACHINE ================= */
    public enum TestType {
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    }

    // Default test
    private TestType selectedTest = TestType.QUASISTATIC_FORWARD;
    private boolean isRunning = false;

    /* ================= HARDWARE ================= */
    private DcMotorEx motor;
    private DcMotorEx motor2;
    private VoltageSensor voltageSensor;
    private List<LynxModule> allHubs;

    /* ================= LOGGING ================= */
    private FileWriter logWriter;
    private ElapsedTime timer = new ElapsedTime();
    private final StringBuilder logBuffer = new StringBuilder();

    @Override
    public void init() {
        // 1. Setup Bulk Caching (CRITICAL for high frequency logging)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Setup Hardware
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME2);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 3. Configure Motor for pure voltage control
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        telemetry.addLine("Initialized.");
        telemetry.addLine("Use D-PAD UP/DOWN to select test.");
        telemetry.addLine("Press A to confirm and generate file.");
    }

    @Override
    public void init_loop() {
        // Simple UI to select test without recompiling
        if (gamepad1.dpad_up) {
            selectedTest = TestType.QUASISTATIC_FORWARD;
        } else if (gamepad1.dpad_down) {
            selectedTest = TestType.QUASISTATIC_REVERSE;
        } else if (gamepad1.dpad_right) {
            selectedTest = TestType.DYNAMIC_FORWARD;
        } else if (gamepad1.dpad_left) {
            selectedTest = TestType.DYNAMIC_REVERSE;
        }

        telemetry.addData("Selected Test", selectedTest);
        telemetry.addLine("Press START to begin.");
        telemetry.update();
    }

    @Override
    public void start() {
        // create a unique file name so we don't overwrite previous runs
        String filename = String.format(Locale.US, "/sdcard/FIRST/sysid_%s_%d.csv",
                selectedTest.toString(), System.currentTimeMillis());

        try {
            logWriter = new FileWriter(filename);
            // We ONLY log raw data. Acceleration is calculated in post-processing.
            logWriter.write("timestamp,voltage,position,velocity\n");
        } catch (IOException e) {
            throw new RuntimeException("Failed to open log file", e);
        }

        timer.reset();
        isRunning = true;
    }

    @Override
    public void loop() {
        if (!isRunning) return;

        // Clear bulk cache to get fresh data once per loop
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        double time = timer.seconds();

        // Read Battery Voltage (this fluctuates under load, so we need it every loop)
        double batteryVoltage = voltageSensor.getVoltage();

        double targetVoltage = 0.0;

        // Calculate Target Voltage based on Test Mode
        switch (selectedTest) {
            case QUASISTATIC_FORWARD:
                targetVoltage = RAMP_RATE * time;
                break;
            case QUASISTATIC_REVERSE:
                targetVoltage = -RAMP_RATE * time;
                break;
            case DYNAMIC_FORWARD:
                targetVoltage = (time > STEP_DELAY) ? STEP_VOLTAGE : 0.0;
                break;
            case DYNAMIC_REVERSE:
                targetVoltage = (time > STEP_DELAY) ? -STEP_VOLTAGE : 0.0;
                break;
        }

        // Clamp to safety limits
        targetVoltage = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, targetVoltage));

        // Convert Voltage to Motor Power (-1.0 to 1.0)
        // Power = DesiredVolts / CurrentBatteryVolts
        double motorPower = targetVoltage / batteryVoltage;

        // Safety clamp for power
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        motor.setPower(motorPower);
        motor2.setPower(motorPower);

        // LOGGING
        // We log the *actual* applied voltage (MotorPower * BatteryVoltage)
        // This accounts for battery sag.
        double appliedVoltage = motorPower * batteryVoltage;
        double position = motor.getCurrentPosition();
        double velocity = motor.getVelocity(); // Ticks per second

        try {
            logWriter.write(String.format(Locale.US, "%.4f,%.4f,%.2f,%.2f\n",
                    time, appliedVoltage, position, velocity));
        } catch (IOException e) {
            telemetry.addData("Error", "Writing failed");
        }

        telemetry.addData("Test", selectedTest);
        telemetry.addData("Applied Volts", appliedVoltage);
        telemetry.addData("Velocity", velocity);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
        try {
            if (logWriter != null) {
                logWriter.flush();
                logWriter.close();
            }
        } catch (IOException e) {
            // ignore
        }
    }
}