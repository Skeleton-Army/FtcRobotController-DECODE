package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.VisionConfig.*;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.skeletonarmy.marrow.OpModeManager;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class Vision extends SubsystemBase {
    private final double METERS_TO_INCHES = 39.37;

    private final PoseTracker poseTracker;
    private final Limelight3A limelight;

    private final TimerEx relocalizeTimer = new TimerEx(RELOCALIZE_COOLDOWN);

    private final List<Consumer<Pose>> onRelocalizeListeners = new ArrayList<>();

    public Vision(HardwareMap hardwareMap, PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(0);
        limelight.start();

        relocalizeTimer.start();
    }

    @Override
    public void periodic() {
        if (relocalizeTimer.isDone()) {
            boolean success = relocalize();
            if (success) {
                relocalizeTimer.restart();
            }
        }
    }

    public Pose getAprilTagPose() {
        Pose FTCRobotPose = poseTracker.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        double orientationDeg = Math.toDegrees(FTCRobotPose.getHeading()) + 180;
        limelight.updateRobotOrientation(orientationDeg);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();

            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

                Pose standardFTCPose = new Pose(x, y, heading).scale(METERS_TO_INCHES);
                Pose pedroPose = FTCCoordinates.INSTANCE.convertToPedro(standardFTCPose);

                return pedroPose;
            }
        }

        return new Pose();
    }

    public boolean relocalize() {
        Pose tagPose = getAprilTagPose();
        double velocity = poseTracker.getVelocity().getMagnitude();
        double angularVelocity = poseTracker.getAngularVelocity();

        if (tagPose.roughlyEquals(new Pose(), 0.001)) return false;
        if (Math.abs(velocity) > VELOCITY_THRESHOLD || Math.abs(angularVelocity) > VELOCITY_THRESHOLD) return false;

        poseTracker.setPose(tagPose);

        for (Consumer<Pose> listener : onRelocalizeListeners) {
            listener.accept(tagPose);
        }

        return true;
    }

    public void addRelocalizationListener(Consumer<Pose> listener) {
        onRelocalizeListeners.add(listener);
    }
}
