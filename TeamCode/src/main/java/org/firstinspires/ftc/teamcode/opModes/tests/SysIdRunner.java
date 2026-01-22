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

@TeleOp(name = "SysId FTC Runner", group = "SysId")
public class SysIdRunner extends OpMode {

    /* ================= CONFIG ================= */
    private static final String MOTOR_1 = "flywheel1";
    private static final String MOTOR_2 = "flywheel2";

    private static final double RAMP_RATE = 0.25;     // V/s
    private static final double STEP_VOLTAGE = 5.5;   // V
    private static final double STEP_DELAY = 1.0;     // s
    private static final double MAX_VOLTAGE = 12.0;
    private static final double IDLE_TIME = 4.0;      // s

    /* ================= STATES ================= */
    private enum State {
        IDLE,
        QS_FWD,
        QS_REV,
        DYN_FWD,
        DYN_REV,
        DONE
    }

    private State state = State.IDLE;
    private State nextState = State.QS_FWD;

    /* ================= HARDWARE ================= */
    private DcMotorEx motor1, motor2;
    private VoltageSensor battery;
    private List<LynxModule> hubs;

    /* ================= LOGGING ================= */
    private FileWriter writer;
    private final ElapsedTime globalTime = new ElapsedTime();
    private final ElapsedTime stateTime = new ElapsedTime();

    private boolean waitingForAdvance = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        motor1 = hardwareMap.get(DcMotorEx.class, MOTOR_1);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.get(DcMotorEx.class, MOTOR_2);
        battery = hardwareMap.voltageSensor.iterator().next();

        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("SysId FTC Ready");
        telemetry.addLine("Press START to begin");
    }

    @Override
    public void start() {
        try {
            writer = new FileWriter(
                    String.format(Locale.US,
                                "/sdcard/FIRST/sysid_full_%d.csv",
                            System.currentTimeMillis()
                    )
            );

            writer.write("timestamp,voltage,position,velocity,sysid-test-state\n");

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        globalTime.reset();
        stateTime.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        if (gamepad1.b) {
            stopMotors();
            state = State.DONE;
        }

        double t = globalTime.seconds();
        double volts = 0.0;
        String sysidState = "none";

        switch (state) {

            case IDLE:
                stopMotors();
                sysidState = "none";

                if (stateTime.seconds() >= IDLE_TIME) {
                    waitingForAdvance = true;
                }

                if (waitingForAdvance && gamepad1.a) {
                    state = nextState;
                    stateTime.reset();
                    waitingForAdvance = false;
                }
                break;

            case QS_FWD:
                sysidState = "quasistatic-forward";
                volts = RAMP_RATE * stateTime.seconds();
                if (volts >= MAX_VOLTAGE) endTest(State.QS_REV);
                break;

            case QS_REV:
                sysidState = "quasistatic-reverse";
                volts = -RAMP_RATE * stateTime.seconds();
                if (-volts >= MAX_VOLTAGE) endTest(State.DYN_FWD);
                break;

            case DYN_FWD:
                sysidState = "dynamic-forward";
                volts = stateTime.seconds() > STEP_DELAY ? STEP_VOLTAGE : 0.0;
                if (stateTime.seconds() > STEP_DELAY + 2.0) endTest(State.DYN_REV);
                break;

            case DYN_REV:
                sysidState = "dynamic-reverse";
                volts = stateTime.seconds() > STEP_DELAY ? -STEP_VOLTAGE : 0.0;
                if (stateTime.seconds() > STEP_DELAY + 2.0) endTest(State.DONE);
                break;

            case DONE:
                stopMotors();
                telemetry.addLine("SysId Complete");
                telemetry.update();
                return;
        }

        applyVoltage(volts);
        log(t, volts, sysidState);

        telemetry.addData("State", state);
        telemetry.addData("Advance", waitingForAdvance ? "Press A" : "-");
        telemetry.update();
    }

    private void endTest(State next) {
        stopMotors();
        nextState = next;
        state = State.IDLE;
        stateTime.reset();
    }

    private void applyVoltage(double volts) {
        volts = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, volts));
        double power = volts / battery.getVoltage();
        power = Math.max(-1.0, Math.min(1.0, power));
        motor1.setPower(power);
        motor2.setPower(power);
    }

    private void stopMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    private void log(double t, double volts, String state) {
        try {
            writer.write(String.format(
                    Locale.US,
                    "%.4f,%.3f,%.2f,%.2f,%s\n",
                    t,
                    volts,
                    (float)motor1.getCurrentPosition(),
                    motor1.getVelocity(),
                    String.format("%s", state)
            ));
        } catch (IOException ignored) {}
    }

    @Override
    public void stop() {
        stopMotors();
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {}
    }
}
