package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "FTC SysId Logger", group = "SysId")
public class SysIdRunner extends OpMode {

    /* ================= CONFIG ================= */

    private static final String MOTOR_1 = "flywheel1";
    private static final String MOTOR_2 = "flywheel2";

    private static final double RAMP_RATE = 0.25;     // volts/sec (quasistatic)
    private static final double STEP_VOLTAGE = 9.0;   // volts (dynamic)
    private static final double MAX_VOLTAGE = 12.0;

    private static final double IDLE_TIME = 0.75;    // seconds
    private static final double QS_TIME = 70;       // seconds
    private static final double DYN_TIME = 15;     // seconds

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

    private FileWriter writer;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime globalTimer = new ElapsedTime();

    private boolean lastA = false;

    /* ================= INIT ================= */

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
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("SysId Ready");
        telemetry.addLine("Press A to advance tests");
        stateIndex = -1;
        advanceState(); // moves to index 0 safely
    }

    @Override
    public void start() {
        try {
            writer = new FileWriter(
                    String.format(Locale.US,
                            "/sdcard/FIRST/sysid_ftc_%d_fixed.csv",
                            System.currentTimeMillis())
            );
            writer.write("Timestamp,voltage,position,velocity,sysid-test-state\n");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        globalTimer.reset();
        advanceState();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        double time = globalTimer.seconds();
        double volts = battery.getVoltage();
        double cmdVolts = 0.0;

        switch (currentState) {

            case IDLE:
                cmdVolts = 0.0;
                break;

            case QS_FWD:
                cmdVolts = RAMP_RATE * stateTimer.seconds();
                break;

            case QS_REV:
                cmdVolts = -RAMP_RATE * stateTimer.seconds();
                break;

            case DYN_FWD:
                cmdVolts = STEP_VOLTAGE;
                break;

            case DYN_REV:
                cmdVolts = -STEP_VOLTAGE;
                break;

            case DONE:
                cmdVolts = 0.0;
                break;
        }

        cmdVolts = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, cmdVolts));
        double power = cmdVolts / volts;

        motor1.setPower(power);
        motor2.setPower(power);

        double position = motor1.getCurrentPosition();
        double velocity = motor1.getVelocity();

        try {
            writer.write(String.format(
                    Locale.US,
                    "%.6f,%.4f,%.2f,%.2f,%s\n",
                    time, power * volts, position, velocity, currentState.label
            ));
        } catch (IOException ignored) {}

        telemetry.addData("State", currentState.label);
        telemetry.addData("Time", stateTimer.seconds());
        telemetry.addLine("Press A to continue");
        telemetry.update();

        // Only allow A to advance when we are IDLE
        boolean a = gamepad1.a;

        if (currentState == TestState.IDLE) {
            if (a && !lastA && stateTimer.seconds() >= IDLE_TIME) {
                advanceState();
            }
        } else {
            // Automatically end tests when time expires
            if (stateTimer.seconds() >= minStateTime(currentState)) {
                advanceState();
            }
        }

        lastA = a;

    }

    /* ================= HELPERS ================= */

    private void advanceState() {
        stateIndex++;

        if (stateIndex >= testOrder.length) {
            currentState = TestState.DONE; // or DONE
            return;
        }

        currentState = testOrder[stateIndex];
        stateTimer.reset();
    }

    private double minStateTime(TestState s) {
        switch (s) {
            case IDLE: return IDLE_TIME;
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
            writer.close();
        } catch (IOException ignored) {}
    }
}
