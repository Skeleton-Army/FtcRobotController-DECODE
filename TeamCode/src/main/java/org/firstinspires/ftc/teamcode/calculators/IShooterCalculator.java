package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public interface IShooterCalculator {
    public ShootingSolution getShootingSolution(Pose robotPose, Vector goalVelocity);
}
