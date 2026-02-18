package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision extends SubsystemBase {
    private final PoseTracker poseTracker;
    private final Limelight3A limelight;

    public Vision(HardwareMap hardwareMap, PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose getAprilTagPose() {
        // TODO: Accept pose only if it's from the GOAL's tag

        Pose FTCRobotPose = poseTracker.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        double orientationDeg = Math.toDegrees(FTCRobotPose.getHeading());
        limelight.updateRobotOrientation(orientationDeg);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();

            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

                Pose standardFTCPose = new Pose(x, y, heading);
                Pose pedroPose = FTCCoordinates.INSTANCE.convertToPedro(standardFTCPose);

                return pedroPose;
            }
        }

        return new Pose();
    }
}
