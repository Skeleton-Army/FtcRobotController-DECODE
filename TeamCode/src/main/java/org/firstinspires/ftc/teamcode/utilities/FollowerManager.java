package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

/**
 * Manages the Follower lifecycle across OpModes.
 * Ensures hardware is fresh while maintaining spatial continuity.
 */
public class FollowerManager {
    private static final String TAG = "FollowerManager";
    private static Follower follower;

    public static Follower createPinpointFollower(HardwareMap hardwareMap) {
        return doCreateFollower(() -> Constants.createPinpointFollower(hardwareMap));
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return doCreateFollower(() -> Constants.createFollower(hardwareMap));
    }

    /**
     * Creates a new Follower tied to the current HardwareMap.
     * If a previous Follower exists in memory, it inherits its last known Pose.
     * If not (first run, or the RC app restarted between OpModes), falls back to
     * whatever pose is stored on the Pinpoint sensor's own firmware, which survives
     * app restarts as long as the device hasn't lost power.
     */
    private static Follower doCreateFollower(Supplier<Follower> method) {
        Pose lastPose = extractLastPose();
        follower = null; // release the old reference; do not touch its hardware again

        Follower newFollower;
        try {
            newFollower = method.get();
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Failed to construct new Follower");
            throw e;
        }

        // Configure bulk-read scope BEFORE any pose read/write below touches the
        // device, so the very first hardware transaction already uses the trimmed
        // register scope instead of the driver's default.
        configureBulkReadScope(newFollower);

        if (lastPose == null) {
            lastPose = readRawPinpointPose(newFollower);
        }

        if (lastPose != null) {
            newFollower.setPose(lastPose);
        } else {
            RobotLog.ww(TAG, "No prior pose available - Follower starting at its default pose");
        }

        follower = newFollower;
        return follower;
    }

    /**
     * Pulls the last known pose from the previous Follower's localizer, if one exists
     * in memory. Reads only cached in-memory state - never forces a fresh hardware
     * read on the previous OpMode's device, since it may already be torn down.
     */
    private static Pose extractLastPose() {
        if (follower == null) return null;

        Localizer localizer = follower.poseTracker.getLocalizer();

        try {
            if (localizer instanceof FusionLocalizer) {
                return ((FusionLocalizer) localizer).getPose();
            }
            if (localizer instanceof PinpointLocalizer) {
                return localizer.getPose();
            }
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Failed to read pose from previous Follower's localizer");
            return null;
        }

        RobotLog.ww(TAG, "Previous localizer type not recognized (%s) - pose continuity lost",
                localizer == null ? "null" : localizer.getClass().getSimpleName());
        return null;
    }

    /**
     * Reads whatever pose is currently stored on the Pinpoint sensor itself, using the
     * NEW Follower's (currently valid) hardware. This is the recovery path for when
     * static in-memory state didn't survive - e.g. the RC app restarted between
     * autonomous and teleop.
     */
    private static Pose readRawPinpointPose(Follower newFollower) {
        Localizer effective = resolveToPinpointLocalizer(newFollower);
        if (effective == null) return null;

        try {
            GoBildaPinpointDriver pinpoint = ((PinpointLocalizer) effective).getPinpoint();
            pinpoint.update();
            Pose rawPose = effective.getPose();
            RobotLog.ii(TAG, "Recovered pose from Pinpoint hardware: %s", rawPose);
            return rawPose;
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Failed to read raw pose from Pinpoint hardware");
            return null;
        }
    }

    private static void configureBulkReadScope(Follower newFollower) {
        Localizer effective = resolveToPinpointLocalizer(newFollower);
        if (effective == null) return;

        GoBildaPinpointDriver pinpoint = ((PinpointLocalizer) effective).getPinpoint();
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

    private static Localizer resolveToPinpointLocalizer(Follower f) {
        Localizer localizer = f.poseTracker.getLocalizer();
        Localizer effective = (localizer instanceof FusionLocalizer)
                ? ((FusionLocalizer) localizer).getDeadReckoning()
                : localizer;
        return (effective instanceof PinpointLocalizer) ? effective : null;
    }
}