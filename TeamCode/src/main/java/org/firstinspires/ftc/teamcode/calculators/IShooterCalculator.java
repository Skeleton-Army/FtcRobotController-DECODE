package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public interface IShooterCalculator {
    ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Vector goalVelocity);
}
