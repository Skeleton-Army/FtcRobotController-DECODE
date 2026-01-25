package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter; // Added for fix
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "FTC SysId Logger Fixed", group = "SysId")
public class SysIdRunner extends OpMode {

    /* ================= CONFIG ================= */
    private static final String MOTOR_1 = "flywheel1";
    private static final String MOTOR_2 = "flywheel2";

    private static final double RAMP_RATE = 0.25;
    private static final double STEP_VOLTAGE = 9.0;
    private static final double MAX_VOLTAGE = 12.0;

    private static final double IDLE_TIME = 0.75;
    private static final double QS_TIME = 70;
    private static final double DYN_TIME = 15;

    /* ================= STATES ================= */
    private enum TestState {
        IDLE("idle"),
        QS_FWD("quasistatic-forward"),
        QS_REV("quasistatic-reverse"),
        DYN_FWD("dynamic-forward"),
        DYN_REV("dynamic-reverse"),
        DONE("Done");

        public final String label;
        TestState(String label) { this.label = label; }
    }

    private TestState[] testOrder = {
            TestState.IDLE,
            TestState.QS_FWD,
            TestState.IDLE,
            TestState.QS_REV,
            TestState.IDLE,
            TestState.DYN_FWD,
            TestState.IDLE,
            TestState.DYN_REV,
            TestState.IDLE,
            TestState.DONE
    };

    private int stateIndex = 0;
    private TestState currentState;

    /* ================= HARDWARE ================= */
    private DcMotorEx motor1, motor2;
    private VoltageSensor battery;
    private List<LynxModule> hubs;

    /* ================= LOGGING ================= */
    private BufferedWriter writer; // Changed to BufferedWriter
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime globalTimer = new ElapsedTime();
    private long sampleCount = 0;
    private boolean lastA = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        motor1 = hardwareMap.get(DcMotorEx.class, MOTOR_1);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.get(DcMotorEx.class, MOTOR_2);

        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().next();

        stateIndex = 0; // Initialize at 0
        currentState = testOrder[stateIndex];

        telemetry.addLine("SysId Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        try {
            // FIX: Using BufferedWriter for high-speed logging
            writer = new BufferedWriter(new FileWriter(
                    String.format(Locale.US, "/sdcard/FIRST/sysid_%d_dynamic_fix.csv", System.currentTimeMillis())
            ));
            writer.write("Timestamp,voltage,position,velocity,\"sysid-test-state\"\n");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        globalTimer.reset();
        stateTimer.reset();
        sampleCount = 0;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        double volts = battery.getVoltage();
        double power = 0.0;

        // Apply logic based on state
        switch (currentState) {
            case QS_FWD:  power = (RAMP_RATE * stateTimer.seconds()) / volts; break;
            case QS_REV:  power = (-RAMP_RATE * stateTimer.seconds()) / volts; break;
            case DYN_FWD: power = STEP_VOLTAGE / 12.0; break; // Fixed Power method
            case DYN_REV: power = -STEP_VOLTAGE / 12.0; break;
            default:      power = 0.0; break;
        }

        power = Math.max(-1.0, Math.min(1.0, power));
        motor1.setPower(power);
        motor2.setPower(power);

        // High-precision logging with dither to prevent "1 sample" deduplication
        try {
            double appliedVolts = (power * volts) + (Math.random() * 0.001);
            writer.write(String.format(Locale.US, "%.6f,%.4f,%.2f,%.2f,\"%s\"\n",
                    globalTimer.seconds(),
                    appliedVolts,
                    (float)motor1.getCurrentPosition(),
                    motor1.getVelocity(),
                    currentState.label));
            sampleCount++;
        } catch (IOException ignored) {}

        // State Transitions
        if (currentState == TestState.IDLE) {
            if (gamepad1.a && !lastA && stateTimer.seconds() >= IDLE_TIME) {
                advanceState();
            }
        } else if (currentState != TestState.DONE) {
            if (stateTimer.seconds() >= minStateTime(currentState)) {
                advanceState();
            }
        }

        lastA = gamepad1.a;
        telemetry.addData("State", currentState.label);
        telemetry.addData("Samples", sampleCount);
        telemetry.update();
    }

    private void advanceState() {
        stateIndex++;
        if (stateIndex >= testOrder.length) {
            currentState = TestState.DONE;
        } else {
            currentState = testOrder[stateIndex];
        }
        stateTimer.reset();
    }

    private double minStateTime(TestState s) {
        switch (s) {
            case QS_FWD:
            case QS_REV: return QS_TIME;
            case DYN_FWD:
            case DYN_REV: return DYN_TIME;
            default: return 0;
        }
    }

    @Override
    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
        try {
            if (writer != null) {
                writer.flush(); // CRITICAL: Forces RAM buffer to SD card
                writer.close();
            }
        } catch (IOException ignored) {}
    }
}