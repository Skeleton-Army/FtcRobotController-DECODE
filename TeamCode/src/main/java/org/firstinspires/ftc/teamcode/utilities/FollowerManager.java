package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
            follower.updatePose(); // read while old driver still has clean I2C access
            lastPose = follower.getPose();
            follower = null; // release old driver before new one reconfigures the device
        }

        Follower newFollower = Constants.createFollower(hardwareMap);

        if (lastPose != null) {
            newFollower.setPose(lastPose);
            newFollower.updatePose();
        }

        follower = newFollower;
        return follower;
    }
}
