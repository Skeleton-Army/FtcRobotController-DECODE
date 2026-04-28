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

            if (localizer instanceof PinpointLocalizer) {
                PinpointLocalizer pinpointLocalizer = (PinpointLocalizer) localizer;
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

        follower = newFollower;
        return follower;
    }
}
