package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_KD;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_KI;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_KP;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_KS;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_KV;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_MOTOR;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_NAME;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_MAX;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_MIN;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.HOOD_NAME;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_KP;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_NAME;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GOAL_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GOAL_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.DISTANCE_TO_BOT_CENTER;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

@Config
public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    public final Motor turret;
    private final ServoEx hood;

    private double velocity;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, FLYWHEEL_NAME, FLYWHEEL_MOTOR);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);

        turret = new Motor(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setPositionCoefficient(TURRET_KP);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse(Math.PI * 2 / (turret.getCPR() * GEAR_RATIO));

        hood = new ServoEx(hardwareMap, HOOD_NAME, HOOD_MIN, HOOD_MAX);
    }

    private void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    private void setVerticalAngle(double angleRad) {
        hood.set(angleRad);
    }

    private void setHorizontalAngle(double target) {
        double normalizedTarget = MathFunctions.normalizeAngle(target);
        turret.setTargetDistance(normalizedTarget);
    }

    public void updateHorizontalAngle() {
        double x = poseTracker.getPose().getX();
        double y = poseTracker.getPose().getY();
        double heading = poseTracker.getPose().getHeading();

        setHorizontalAngle(getTurretAngle(x, y, heading));
    }

    public void updateVerticalAngle() {
        // TODO: Do calculations and set vertical angle to GOAL
    }

    @Override
    public void periodic() {
        flywheel.setVelocity(velocity, AngleUnit.RADIANS);
        turret.set(1);
    }

    public static double getTurretAngle(double x, double y, double heading) {
        double turretX = x + DISTANCE_TO_BOT_CENTER * Math.cos(heading);
        double turretY = y + DISTANCE_TO_BOT_CENTER * Math.sin(heading);

        double angle = Math.atan2(GOAL_Y - turretY, GOAL_X - turretX);
        double target = angle - heading;

        return MathFunctions.normalizeAngle(target);
    }
}
