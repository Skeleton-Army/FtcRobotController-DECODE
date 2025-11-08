package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;

@Config
public class Shooter extends SubsystemBase {
    private final PoseTracker poseTracker;

    private final MotorEx flywheel;
    private final Motor turret;
    private final ServoEx hood;
    private final CRServoEx transfer;

    private double velocity;

    public Shooter(final HardwareMap hardwareMap, final PoseTracker poseTracker) {
        this.poseTracker = poseTracker;

        flywheel = new MotorEx(hardwareMap, FLYWHEEL_NAME, FLYWHEEL_MOTOR);
        flywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheel.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV);
        flywheel.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheel.setInverted(FLYWHEEL_INVERTED);

        turret = new Motor(hardwareMap, TURRET_NAME, ShooterConfig.TURRET_MOTOR);
        turret.setPositionCoefficient(TURRET_KP);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setDistancePerPulse((Math.PI * 2) / (turret.getCPR() * GEAR_RATIO));
        turret.setTargetDistance(0);

        hood = new ServoEx(hardwareMap, HOOD_NAME);
        setVerticalAngle(0);

        transfer = new CRServoEx(hardwareMap, TRANSFER_NAME);
        setHorizontalAngle(0);

        setRPM(FLYWHEEL_TARGET);
    }

    @Override
    public void periodic() {
        flywheel.setVelocity(velocity);
        turret.set(1);
    }

    public void toggleTransfer(boolean isOn) {
        transfer.set(isOn ? TRANSFER_POWER : 0);
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

    public double getRPM() {
        double motorTPS = flywheel.getCorrectedVelocity();
        return (motorTPS * 60.0) / flywheel.getCPR();
    }

    public double getRawHoodPosition() {
        return hood.getRawPosition();
    }

    public double getTurretPosition() {
        return turret.getCurrentPosition();
    }

    private void setRPM(double rpm) {
        this.velocity = (rpm * flywheel.getCPR()) / 60.0;
    }

    public void setRawHoodPosition(double angle) {
        hood.set(angle + HOOD_POSSIBLE_MIN);
    }

    public void setVerticalAngle(double angle) {
        double normalized = (angle - HOOD_MIN) / (HOOD_MAX - HOOD_MIN);
        setRawHoodPosition(normalized);
    }

    public void setHorizontalAngle(double targetAngleRad) {
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
