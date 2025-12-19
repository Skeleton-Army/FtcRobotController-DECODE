package org.firstinspires.ftc.teamcode.utilities;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;

import java.text.SimpleDateFormat;
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

        initialize();
        startLogger();

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
                Logger.end();

                end();
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
        Logger.addDataReceiver(new RLOGServer());
        Logger.start();
    }
}
