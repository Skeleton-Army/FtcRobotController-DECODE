package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.follower.Follower;
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
        // 1. Capture the latest state from the previous OpMode's hardware
        if (follower != null) {
            try {
                follower.update();
            } catch (Exception e) {
                // If the I2C bus is already closed, we just use the last cached pose
            }
        }

        // 2. Create the fresh hardware instance
        Follower newFollower = Constants.createFollower(hardwareMap);

        // 3. Handoff the pose
        if (follower != null) {
            newFollower.setPose(follower.getPose());
        }

        follower = newFollower;
        return follower;
    }
}
