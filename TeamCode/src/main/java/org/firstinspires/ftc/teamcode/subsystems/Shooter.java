package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter extends SubsystemBase {
    public static double kP = 1;
    public static double tolerance = 0.01;

    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    public final Motor turret;
    private final ServoEx hood;

    private double velocity;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, "flywheel", MotorEx.GoBILDA.BARE);
        flywheel.setVeloCoefficients(1, 0, 0);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);

        turret = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_435);
        turret.setPositionCoefficient(kP);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse(Math.PI * 2 / (Motor.GoBILDA.RPM_435.getCPR() * (200 / 30)));
        turret.setPositionTolerance(tolerance);

        hood = new ServoEx(hardwareMap, "hood", 0, Math.PI / 2);
    }

    private void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    private void setVerticalAngle(double angleRad) {
        hood.set(angleRad);
    }

    private void setHorizontalAngle(double target) {
        turret.setTargetDistance(target);
    }

    public double getTurretAngle() {
        double GOAL_X = 0;
        double GOAL_Y = 140;
        double DISTANCE_TO_BOT_CENTER = 0;

        double ROBOT_X = poseTracker.getPose().getX();
        double ROBOT_Y = poseTracker.getPose().getY();
        double ROBOT_HEADING = poseTracker.getPose().getHeading();

        return Math.atan2(GOAL_Y - ROBOT_Y - DISTANCE_TO_BOT_CENTER * Math.sin(ROBOT_HEADING), GOAL_X - ROBOT_X - DISTANCE_TO_BOT_CENTER * Math.cos(ROBOT_HEADING)) - ROBOT_HEADING;
    }

    public void updateHorizontalAngle() {
        setHorizontalAngle(getTurretAngle());
    }

    public void updateVerticalAngle() {
        // TODO: Do calculations and set vertical angle to GOAL
    }

    @Override
    public void periodic() {
        flywheel.setVelocity(velocity, AngleUnit.RADIANS);
        turret.set(1);
    }
}
