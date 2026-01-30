package org.firstinspires.ftc.teamcode.calculators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.consts.ShooterCoefficients;
import org.firstinspires.ftc.teamcode.consts.ShooterConsts;

public class NoAirCalculator implements IShooterCalculator{

    Vector robotToGoalVector = new Vector();
    double goalXOffset = 0;
    double goalYOffset = 0;
    protected int velocityToRPM(double velocity) {
        double[] velCoeffs = ShooterCoefficients.VEL_COEFFS;
        double result = 0.0;
        for (int exponent = 0; exponent < velCoeffs.length; exponent++) {
            result += velCoeffs[exponent] * Math.pow(velocity, velCoeffs.length - exponent - 1);
        }
        return (int)result;
    }
    @Override
    public ShootingSolution getShootingSolution(Pose robotPose, Pose goalPose, Pose turretGoalPose, Vector robotVel, double angularVel, int flywheelRPM) {

        robotToGoalVector.setOrthogonalComponents(goalPose.getX() - robotPose.getX()
                + goalXOffset, goalPose.getY() - robotPose.getY() + goalYOffset);

        //constants
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConsts.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConsts.SCORE_HEIGHT;
        double a = ShooterConsts.SCORE_ANGLE;

        //calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), ShooterConfig.HOOD_MAX,
                ShooterConfig.HOOD_MIN);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        //get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = robotVel;

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), ShooterConfig.HOOD_MAX,
                ShooterConfig.HOOD_MIN);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = robotPose.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset;

        return new ShootingSolution(
                MathFunctions.normalizeAngle(turretAngle),
                hoodAngle,
                velocityToRPM(flywheelSpeed)
        );

    }
}
