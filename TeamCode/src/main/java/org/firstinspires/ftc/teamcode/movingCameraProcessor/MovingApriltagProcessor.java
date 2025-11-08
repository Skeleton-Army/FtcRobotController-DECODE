package org.firstinspires.ftc.teamcode.movingCameraProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
import org.openftc.apriltag.AprilTagDetectorJNI;

public abstract class MovingApriltagProcessor extends AprilTagProcessor {
   public abstract void updatePos(Position deltaPos, YawPitchRollAngles newAngles);
    public enum PoseSolver
    {
        APRILTAG_BUILTIN(-1),
        OPENCV_ITERATIVE(Calib3d.SOLVEPNP_ITERATIVE),
        OPENCV_SOLVEPNP_EPNP(Calib3d.SOLVEPNP_EPNP),
        OPENCV_IPPE(Calib3d.SOLVEPNP_IPPE),
        OPENCV_IPPE_SQUARE(Calib3d.SOLVEPNP_IPPE_SQUARE),
        OPENCV_SQPNP(Calib3d.SOLVEPNP_SQPNP);

        final int code;

        PoseSolver(int code)
        {
            this.code = code;
        }
    }
    public enum TagFamily
    {
        TAG_36h11(AprilTagDetectorJNI.TagFamily.TAG_36h11),
        TAG_25h9(AprilTagDetectorJNI.TagFamily.TAG_25h9),
        TAG_16h5(AprilTagDetectorJNI.TagFamily.TAG_16h5),
        TAG_standard41h12(AprilTagDetectorJNI.TagFamily.TAG_standard41h12);

        final AprilTagDetectorJNI.TagFamily ATLibTF;

        TagFamily(AprilTagDetectorJNI.TagFamily ATLibTF)
        {
            this.ATLibTF = ATLibTF;
        }
    }

    public abstract void setPoseSolver(PoseSolver poseSolver);
}
