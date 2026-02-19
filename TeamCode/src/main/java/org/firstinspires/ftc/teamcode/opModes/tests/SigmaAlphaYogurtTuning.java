package org.firstinspires.ftc.teamcode.opModes.tests;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utilities.ComplexOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@TeleOp
public class SigmaAlphaYogurtTuning extends ComplexOpMode {
    Follower follower;
    List<Yogurt> yogurtList;
    File csv;
    TimerEx timerEx;
    double previousSample = 0;
    Shooter shooter;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(180)));

        yogurtList = new ArrayList<>();

        shooter = new Shooter(hardwareMap, follower.getPoseTracker(), new ShooterCalculator(ShooterCoefficients.HOOD_COEFFS), Alliance.BLUE);

        csv = new File(String.format("%s/FIRST/pinpointNoMove-%s.csv",
                Environment.getExternalStorageDirectory().getPath(),getClass().getSimpleName()));

       if (!csv.exists()) {
           try {
               csv.createNewFile();
           } catch (IOException e) {
               throw new RuntimeException(e);
           }
       } else {
           try (PrintWriter writer = new PrintWriter(csv)){
                writer.write("");
           } catch (FileNotFoundException e) {
               throw new RuntimeException(e);
           }
       }

        try (FileWriter fileWriter = new FileWriter(csv)) {
            fileWriter.append("t,x,y,Heading\n");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        timerEx = new TimerEx(2.5, TimeUnit.MINUTES);
        timerEx.start();
    }

    @Override
    public void run() {
        follower.update();
        double runtime = getRuntime(); //to make sure using same data
        if (runtime >= previousSample + 1) {
            yogurtList.add(new Yogurt(runtime, follower.getPose()));
            previousSample = runtime;
        }
        telemetry.addData("Time remaining", timerEx.getRemaining());
        if (timerEx.isDone()) {
            requestOpModeStop();
        }
        telemetry.update();
    }

    @Override
    public void end() {
        try(FileWriter fileWriter = new FileWriter(csv)) {
            for (Yogurt yogurt : yogurtList) {
                fileWriter.append(String.format(Locale.getDefault(),"%f,%f,%f,%f\n",
                        yogurt.getSampleTime(), yogurt.getX(), yogurt.getY(), yogurt.getHead()));
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
