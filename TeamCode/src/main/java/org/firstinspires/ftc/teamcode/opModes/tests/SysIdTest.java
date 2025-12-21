package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name = "SysId Flywheel", group = "SysId")
public class SysIdTest extends OpMode {

    /* ================= CONFIG ================= */

    private static final String MOTOR_NAME = "flywheel";

    // Encoder config
    private static double TICKS_PER_REV; // change if needed
    private static final boolean USE_RPM = true;

    // Quasistatic
    private static final double RAMP_RATE = 0.25; // volts/sec
    private static final double MAX_VOLTAGE = 12.0;

    // Dynamic
    private static final double STEP_VOLTAGE = 6.0;
    private static final double STEP_DELAY = 0.5;

    /* ================= ENUM ================= */

    public enum TestMode {
        DISABLED,
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    }

    private TestMode testMode = TestMode.QUASISTATIC_FORWARD;

    /* ================= HARDWARE ================= */

    private DcMotorEx motor;
    private VoltageSensor voltageSensor;

    /* ================= LOGGING ================= */

    private FileWriter logWriter;
    private ElapsedTime timer = new ElapsedTime();

    private double lastVelocity = 0.0;
    private double lastTimestamp = 0.0;

    /* ================= INIT ================= */

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        TICKS_PER_REV = motor.getMotorType().getTicksPerRev(); // getting the ticks per rev by the motor

        try {
            logWriter = new FileWriter(
                    "/sdcard/FIRST/sysid_flywheel.csv"
            );
            logWriter.write("time,voltage,velocity,acceleration\n");
        } catch (IOException e) {
            throw new RuntimeException("Failed to open SysId log file", e);
        }

        telemetry.addLine("SysId Flywheel Ready");
        telemetry.addLine("EDIT testMode in code before running!");
    }

    @Override
    public void start() {
        timer.reset();
        lastTimestamp = 0.0;
        lastVelocity = 0.0;
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {
        double time = timer.seconds();
        double batteryVoltage = voltageSensor.getVoltage();

        double commandedVoltage = 0.0;

        switch (testMode) {
            case QUASISTATIC_FORWARD:
                commandedVoltage = RAMP_RATE * time;
                break;

            case QUASISTATIC_REVERSE:
                commandedVoltage = -RAMP_RATE * time;
                break;

            case DYNAMIC_FORWARD:
                commandedVoltage = (time > STEP_DELAY) ? STEP_VOLTAGE : 0.0;
                break;

            case DYNAMIC_REVERSE:
                commandedVoltage = (time > STEP_DELAY) ? -STEP_VOLTAGE : 0.0;
                break;

            case DISABLED:
            default:
                commandedVoltage = 0.0;
        }

        commandedVoltage = clamp(commandedVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);

        double motorPower = commandedVoltage / batteryVoltage;
        motor.setPower(motorPower);

        double velocity = getVelocity();
        double accel = computeAcceleration(velocity, time);

        log(time, commandedVoltage, velocity, accel);

        telemetry.addData("Mode", testMode);
        telemetry.addData("Voltage (V)", commandedVoltage);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Accel", accel);
    }

    /* ================= STOP ================= */

    @Override
    public void stop() {
        motor.setPower(0.0);
        try {
            logWriter.close();
        } catch (IOException ignored) {}
    }

    /* ================= HELPERS ================= */

    private double getVelocity() {
        double ticksPerSec = motor.getVelocity();
        double revsPerSec = ticksPerSec / TICKS_PER_REV;

        if (USE_RPM) {
            return revsPerSec * 60.0;
        } else {
            return revsPerSec * 2.0 * Math.PI; // rad/s
        }
    }

    private double computeAcceleration(double velocity, double time) {
        double dt = time - lastTimestamp;
        double accel = dt > 0 ? (velocity - lastVelocity) / dt : 0.0;

        lastVelocity = velocity;
        lastTimestamp = time;
        return accel;
    }

    private void log(double time, double voltage, double velocity, double accel) {
        try {
            logWriter.write(String.format(
                    Locale.US,
                    "%.4f,%.4f,%.4f,%.4f\n",
                    time, voltage, velocity, accel
            ));
        } catch (IOException ignored) {}
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
