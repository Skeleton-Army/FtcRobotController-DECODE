package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.TimestampedData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;
import org.psilynx.psikit.core.Logger;

import java.lang.reflect.Method;

@TeleOp(name = "Epoch Timestamp Sync2", group = "Telemetry")
public class EpochTimestampSyncOpMode2 extends ComplexOpMode {

    private TimestampedPinpoint pinpoint;
    private Limelight3A limelight;

    // The conversion offset to shift Monotonic Nanos to Epoch Milliseconds
    private long nanoTimeToEpochMillisOffset = 0;

    @Override
    public void initialize() {
        // 1. Calculate the real-time synchronization offset between clocks
        long epochMillisSample = System.currentTimeMillis();
        long nanoTimeSample = System.nanoTime();

        // This converts any System.nanoTime() reading directly into an Epoch Millisecond timestamp
        nanoTimeToEpochMillisOffset = epochMillisSample - (nanoTimeSample / 1_000_000);

        // 2. Safely initialize your hardware devices
        I2cDeviceSynchSimple rawI2cClient = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint").getDeviceClient();
        pinpoint = new TimestampedPinpoint(rawI2cClient);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        pinpoint.resetPosAndIMU();
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Clocks Synchronized Natively!");
    }

    @Override
    public void run() {
        // --- 1. PINPOINT UPDATE & TIMESTAMPS ---
        pinpoint.update();

        // Grab the raw boot-based nanosecond timestamp from the hardware bus
        long pinpointArrivalNanos = pinpoint.getLatestArrivalNanos();

        // Convert to Epoch Milliseconds using our calculation offset
        long pinpointArrivalEpochMs = (pinpointArrivalNanos / 1_000_000) + nanoTimeToEpochMillisOffset;

        // --- 2. LIMELIGHT UPDATE & TIMESTAMPS ---
        long limelightCaptureEpochMs = getResultTimestamp();

        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid())
        {
            LLResult result = limelight.getLatestResult();
        }
        // --- 3. TELEMETRY OUTPUT ---
        telemetry.addLine("=== EPOCH SYNCHRONIZED TIMESTAMPS ===");
        telemetry.addData("Pinpoint Hardware Time (ms UTC)", pinpointArrivalEpochMs);
        telemetry.addData("Limelight Capture Time (ms UTC)", limelightCaptureEpochMs);

        long trueSensorSkewMs = 0;
        if (limelightCaptureEpochMs != Long.MIN_VALUE) {
            // Because they are on the exact same scale, this subtraction represents absolute latency
            trueSensorSkewMs = pinpointArrivalEpochMs - limelightCaptureEpochMs;
            telemetry.addData("Visual Age relative to Odo (ms)", trueSensorSkewMs);
        } else {
            telemetry.addData("Limelight Status", "No Target Detected");
        }

        telemetry.addLine("\n=== ODOMETRY COORDINATES ===");
        telemetry.addData("X (mm)", pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Y (mm)", pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.update();

        Logger.recordOutput("Pinpoint x", pinpoint.getPosition().getX(DistanceUnit.INCH));
        Logger.recordOutput("Pinpoint y", pinpoint.getPosition().getY(DistanceUnit.INCH));
        Logger.recordOutput("Pinpoint heading", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

        Logger.recordOutput("Pinpoint Hardware Time (ms UTC)", "" + pinpointArrivalEpochMs);
        Logger.recordOutput("Limelight Capture Time (ms UTC)", "" + limelightCaptureEpochMs);
        Logger.recordOutput("skew/difference", trueSensorSkewMs);
    }

    /**
     * @return system timestamp of the Limelight result in milliseconds
     * since Jan 1st, 1970 00:00(UTC)
     */
    public long getResultTimestamp() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult == null || !llResult.isValid()) {
            return Long.MIN_VALUE;
        }

        return System.currentTimeMillis() - (long) llResult.getStaleness();
    }

    // =========================================================================
    // THE SAFE EXTENSION SUBCLASS (No reference leak / Multi-run safe)
    // =========================================================================
    public static class TimestampedPinpoint extends GoBildaPinpointDriver {
        private final I2cDeviceSynchSimple rawClient;
        private volatile long latestArrivalNanos = 0;
        private Method hardwareReadMethod = null;

        public TimestampedPinpoint(I2cDeviceSynchSimple deviceClient) {
            super(deviceClient, false);
            this.rawClient = deviceClient;

            try {
                hardwareReadMethod = rawClient.getClass().getMethod("readTimeStamped", int.class, int.class);
                hardwareReadMethod.setAccessible(true);
            } catch (Exception e) {
                hardwareReadMethod = null;
            }
        }

        public long getLatestArrivalNanos() {
            return latestArrivalNanos;
        }

        @Override
        public void update() {
            if (hardwareReadMethod != null) {
                try {
                    TimestampedData packet = (TimestampedData) hardwareReadMethod.invoke(rawClient, 0x02, 40);
                    if (packet != null) {
                        this.latestArrivalNanos = packet.nanoTime;
                    }
                } catch (Exception ignored) {}
            } else {
                this.latestArrivalNanos = System.nanoTime();
            }
            super.update();
        }
    }
}