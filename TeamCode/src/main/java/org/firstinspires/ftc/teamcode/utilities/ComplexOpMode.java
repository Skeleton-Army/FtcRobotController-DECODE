package org.firstinspires.ftc.teamcode.utilities;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.teamcode.consts.ShooterLookupTable;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Calendar;
import java.util.List;

/**
 * An OpMode that extends SolversLib's {@link CommandOpMode}
 * to add PsiKit logging and a start function.
 */
public abstract class ComplexOpMode extends LinearOpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Schedules {@link Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link Subsystem} objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        startLogger();
        initCSV();
        initialize();

        // run the scheduler
        try {
            while (opModeInInit()) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                initialize_loop();
            }

            onStart();

            while (!isStopRequested() && opModeIsActive()) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                double beforeUserStart = Logger.getTimestamp();
                Logger.periodicBeforeUser();
                double beforeUserEnd = Logger.getTimestamp();

                CommandScheduler.getInstance().run();
                run();

                double afterUserStart = Logger.getTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );
            }
        } finally {
            try {
                end();

                Thread.sleep(100); // Give the background logger thread 100ms to catch up
                Logger.end();
            } finally {
                reset();
            }
        }
    }

    public abstract void initialize();
    public void initialize_loop() { }
    public void onStart() { }
    public void run() { }
    public void end() { }

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }

    @SuppressLint({"SdCardPath", "SimpleDateFormat"})
    private void startLogger() {
        Calendar calendar = Calendar.getInstance();
        SimpleDateFormat dateFormat = new SimpleDateFormat("dd-MM-yyyy");
        SimpleDateFormat timeFormat = new SimpleDateFormat("HH-mm-ss");
        String date = dateFormat.format(calendar.getTime());
        String time = timeFormat.format(calendar.getTime());

        String className = getClass().getSimpleName();

        // You must create the "logs" folder using ADB if it doesn't already exist
        String folderName = "/sdcard/FIRST/logs/";
        String logFileName = className + "_" + date + "_" + time;

        Logger.addDataReceiver(new RLOGWriter(folderName, logFileName));

        RLOGServer server = new RLOGServer();
        server.start(); // This ensures the ServerThread and broadcastQueue are created
        Logger.addDataReceiver(server);

        Logger.start();
    }

    @SuppressLint("SdCardPath")
    private void initCSV() {
        try {
            String defaultPath = "/sdcard/FIRST/shooterTables/";
            ShooterLookupTable.VALIDITY_TABLE = CsvUtils.getBooleanMatrixFromCsv(defaultPath + "ValidityTable.csv");
            ShooterLookupTable.ANGLE_TABLE = CsvUtils.getDoubleMatrixFromCsv(defaultPath + "AngleTable.csv");
            ShooterLookupTable.VELOCITY_ARRAY = CsvUtils.getDoubleArrayFromCsv(defaultPath + "VelocityArray.csv");
        }
        catch (Exception e) {
            System.err.println("Error loading shooter lookup table CSV files: " + e.getMessage());
            RobotLog.addGlobalWarningMessage("Error loading shooter lookup table CSV files: " + e.getMessage());
            requestOpModeStop();
            // Load Default Tables, false for validity, 0.0 for angle
            for (int i = 0; i < ShooterLookupTable.DIST_STEPS; i++) {
                for (int j = 0; j < ShooterLookupTable.VEL_STEPS; j++) {
                    ShooterLookupTable.VALIDITY_TABLE[i][j] = false;
                    ShooterLookupTable.ANGLE_TABLE[i][j] = 0.0;
                }
            }
        }
    }

    // ----- LOG OVERLOADS -----

    public static void log(String caption, boolean value) {
        OpModeManager.getTelemetry().addData(caption, value);
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, int value) {
        OpModeManager.getTelemetry().addData(caption, value);
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, long value) {
        OpModeManager.getTelemetry().addData(caption, value);
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, float value) {
        OpModeManager.getTelemetry().addData(caption, value);
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, double value) {
        OpModeManager.getTelemetry().addData(caption, value);
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, String value) {
        OpModeManager.getTelemetry().addData(caption, value);
        Logger.recordOutput(caption, value);
    }

    // Standard Arrays
    public static void log(String caption, double[] value) {
        OpModeManager.getTelemetry().addData(caption, Arrays.toString(value));
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, int[] value) {
        OpModeManager.getTelemetry().addData(caption, Arrays.toString(value));
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, boolean[] value) {
        OpModeManager.getTelemetry().addData(caption, Arrays.toString(value));
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, String[] value) {
        OpModeManager.getTelemetry().addData(caption, Arrays.toString(value));
        Logger.recordOutput(caption, value);
    }

    // 2D Arrays
    public static void log(String caption, double[][] value) {
        OpModeManager.getTelemetry().addData(caption, Arrays.deepToString(value));
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, int[][] value) {
        OpModeManager.getTelemetry().addData(caption, Arrays.deepToString(value));
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, java.util.function.DoubleSupplier value) {
        OpModeManager.getTelemetry().addData(caption, value.getAsDouble());
        Logger.recordOutput(caption, value);
    }

    public static void log(String caption, java.util.function.BooleanSupplier value) {
        OpModeManager.getTelemetry().addData(caption, value.getAsBoolean());
        Logger.recordOutput(caption, value);
    }

    // Enums
    public static <E extends Enum<E>> void log(String caption, E value) {
        OpModeManager.getTelemetry().addData(caption, value.toString());
        Logger.recordOutput(caption, value);
    }

    // WPISerializable (For Pose2d, Rotation2d, etc.)
    public static <T extends org.psilynx.psikit.core.wpi.WPISerializable> void log(String caption, T value) {
        OpModeManager.getTelemetry().addData(caption, value.toString());
        Logger.recordOutput(caption, value);
    }

    // Logged Mechanisms
    public static void log(String caption, org.psilynx.psikit.core.mechanism.LoggedMechanism2d value) {
        OpModeManager.getTelemetry().addData(caption, "{Mechanism2d}");
        Logger.recordOutput(caption, value);
    }
}
