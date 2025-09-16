package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public interface IShooterCalculator {
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector3D goalVelocity);
}
