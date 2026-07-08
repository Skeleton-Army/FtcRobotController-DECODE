package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Manages the Follower lifecycle across OpModes.
 * Ensures hardware is fresh while maintaining spatial continuity.
 */
public class FollowerManager {
    private static Follower follower;

    /**
     * Creates a new Follower tied to the current HardwareMap.
     * If a previous Follower exists, it inherits its last known Pose.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        Pose lastPose = null;

        if (follower != null) {
            Localizer localizer = follower.poseTracker.getLocalizer();

            Localizer effectiveLocalizer = localizer;
            if (localizer instanceof FusionLocalizer) {
                FusionLocalizer fusionLocalizer = (FusionLocalizer) localizer;
                lastPose = fusionLocalizer.getPose();
                effectiveLocalizer = fusionLocalizer.getDeadReckoning();
            }

            if (lastPose == null && effectiveLocalizer instanceof PinpointLocalizer) {
                PinpointLocalizer pinpointLocalizer = (PinpointLocalizer) effectiveLocalizer;
                GoBildaPinpointDriver pinpoint = pinpointLocalizer.getPinpoint();

                // Force fresh hardware read
                pinpoint.update();

                // Convert using Pedro's converter
                lastPose = PoseConverter.pose2DToPose(
                        pinpoint.getPosition(),
                        PedroCoordinates.INSTANCE
                );
            }

            follower = null; // release old driver
        }

        Follower newFollower = Constants.createFollower(hardwareMap);

        if (lastPose != null) {
            newFollower.setPose(lastPose);
        }

        Localizer newLocalizer = newFollower.poseTracker.getLocalizer();

        Localizer effectiveNewLocalizer = newLocalizer;
        if (newLocalizer instanceof FusionLocalizer) {
            effectiveNewLocalizer = ((FusionLocalizer) newLocalizer).getDeadReckoning();
        }

        if (effectiveNewLocalizer instanceof PinpointLocalizer) {
            GoBildaPinpointDriver pinpoint = ((PinpointLocalizer) effectiveNewLocalizer).getPinpoint();

            int deviceVersion = pinpoint.getDeviceVersion();
            if (deviceVersion >= 3) {
                pinpoint.setBulkReadScope(
                        GoBildaPinpointDriver.Register.X_POSITION,
                        GoBildaPinpointDriver.Register.Y_POSITION,
                        GoBildaPinpointDriver.Register.H_ORIENTATION,
                        GoBildaPinpointDriver.Register.X_VELOCITY,
                        GoBildaPinpointDriver.Register.Y_VELOCITY,
                        GoBildaPinpointDriver.Register.H_VELOCITY
                );
            }
        }

        follower = newFollower;
        return follower;
    }
}