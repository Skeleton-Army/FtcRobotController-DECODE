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
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_MIN;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_NAME;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GOAL_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.GOAL_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TURRET_OFFSET_Y;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.calculators.IShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShooterCalculator;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.consts.Alliance;
import org.firstinspires.ftc.teamcode.consts.GoalPositions;

import java.io.IOException;

@Config
public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final Motor turret;
    private final ServoEx hood;

    private final IShooterCalculator shooterCalculator;
    private final Alliance alliance;
    private final Pose goalPose;
    /**
     * Flywheel velocity [ticks/second]
     */
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
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));

        hood = new ServoEx(hardwareMap, HOOD_NAME, HOOD_MIN, HOOD_MAX);
        this.shooterCalculator = shooterCalculator;

        this.alliance = alliance;

        this.goalPose = alliance == Alliance.BLUE ? GoalPositions.BLUE_GOAL : GoalPositions.RED_GOAL;
    }

    @Override
    public void periodic() {
        ShootingSolution solution = shooterCalculator.getShootingSolution(poseTracker.getPose(), goalPose, poseTracker.getVelocity());

        setVerticalAngle(solution.getVerticalAngle());
        setVelocity(solution.getVelocity());

        flywheel.setVelocity(velocity, AngleUnit.RADIANS);
        turret.set(1);
        hood.set(this.hoodAngle, AngleUnit.RADIANS);
    }

    public void updateHorizontalAngle() {
        double x = poseTracker.getPose().getX();
        double y = poseTracker.getPose().getY();
        double heading = poseTracker.getPose().getHeading();

        setHorizontalAngle(calculateTurretAngle(x, y, heading));
    }

    public void updateVerticalAngle() {
        // TODO: Do calculations and set vertical angle to GOAL
    }

    private void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    private void setVerticalAngle(double angleRad) {
        hood.set(angleRad);
    }

    private void setHorizontalAngle(double targetAngleRad) {
        double wrapped = wrapToTarget(turret.getDistance(), targetAngleRad, TURRET_MIN, TURRET_MAX);
        turret.setTargetDistance(wrapped);
    }

    // --- CALCULATIONS ---

    public static double calculateTurretAngle(double x, double y, double heading) {
        double turretX = x + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = y + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        double angle = Math.atan2(GOAL_Y - turretY, GOAL_X - turretX);
        double target = angle - heading;

        return MathFunctions.normalizeAngle(target);
    }

    /**
     * Compute the equivalent of {@code target} (mod 2π) that is nearest to {@code current},
     * but force the result into the physical turret limits [min, max].
     *
     * <p><b>Assumption:</b> {@code current} is inside [min, max]. {@code target} is assumed
     * to be in [0, 2π).
     *
     * @param current current angle in radians (must satisfy min <= current <= max)
     * @param target  desired target angle in radians (0 <= target < 2π)
     * @param min     minimal allowed angle (radians)
     * @param max     maximal allowed angle (radians)
     * @return the target equivalent nearest to current, clamped to [min, max]
     */
    public static double wrapToTarget(double current, double target, double min, double max) {
        double closestEquiv = target + 2 * Math.PI * Math.round((current - target) / (2 * Math.PI));
        if (closestEquiv < min) {
            return closestEquiv + 2 * Math.PI;
        }
        if (closestEquiv > max) {
            return closestEquiv - 2 * Math.PI;
        }
        return closestEquiv;
    }
}
