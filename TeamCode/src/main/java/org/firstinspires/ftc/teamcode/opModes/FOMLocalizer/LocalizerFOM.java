package org.firstinspires.ftc.teamcode.opModes.FOMLocalizer;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Objects;

public class LocalizerFOM implements Localizer {
    private final GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private Pose currentVelocity;
    private Pose pinpointPose;
    private Pose limelightPose;
    private Pose robotPose;
    private Limelight3A limelight;
    private Pose3D cameraApriltagPose;
    private FOMCalculator FOMcalculator;

    public LocalizerFOM(HardwareMap hardwareMap, FOMLocalizerConstants constants, Pose setStartPose) {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,constants.hardwareMapName);
        setOffsets(constants.forwardPodY, constants.strafePodX, constants.distanceUnit);

        if(constants.yawScalar.isPresent()) {
            odo.setYawScalar(constants.yawScalar.getAsDouble());
        }

        if(constants.customEncoderResolution.isPresent()) {
            odo.setEncoderResolution(constants.customEncoderResolution.getAsDouble(), constants.distanceUnit);
        } else {
            odo.setEncoderResolution(constants.encoderResolution);
        }

        odo.setEncoderDirections(constants.forwardEncoderDirection, constants.strafeEncoderDirection);

        setStartPose(setStartPose);
        totalHeading = 0;
        pinpointPose = startPose;
        currentVelocity = new Pose();
        previousHeading = setStartPose.getHeading();



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public Pose getPose() {
        return robotPose;
    }
    public Pose getLimelightPose() {
        LLResult result = limelight.getLatestResult();
        double robotYaw = odo.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result.isValid()) {
            // Access general information

            Position botpose_mt2 = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH);
            if (botpose_mt2 != null) {
                YawPitchRollAngles orientation = result.getBotpose_MT2().getOrientation();

                cameraApriltagPose = result.getFiducialResults().get(0).getCameraPoseTargetSpace();

                limelightPose = new Pose(botpose_mt2.x, botpose_mt2.y, orientation.getYaw(AngleUnit.DEGREES));

                limelightPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, botpose_mt2.x, botpose_mt2.y, AngleUnit.DEGREES, orientation.getYaw()), limelightPose.getCoordinateSystem());

            }
            else {
                limelightPose = new Pose(0,0,0);
            }
        }
        return limelightPose;
    }

    public Pose getPinpointPose() {
        return pinpointPose;
    }
    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = pinpointPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odo.update();
        Pose currentPinpointPose = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        // Thank you to GoldenElf58 of FTC Team 16657 for spotting a bug here; it was resolved by adding the turn direction.
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentPinpointPose.getHeading());
        previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
        pinpointPose = currentPinpointPose;
        robotPose = currentPinpointPose; // setting also the calculated robot pose
    }

    @Override
    public void update() {

        odo.update();
        Pose currentPinpointPose = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        // Thank you to GoldenElf58 of FTC Team 16657 for spotting a bug here; it was resolved by adding the turn direction.
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentPinpointPose.getHeading());
        previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
        pinpointPose = currentPinpointPose;

        limelightPose = getLimelightPose(); // updates the limelight pose for calculation

        robotPose = FOMcalculator.calcPos(limelightPose, cameraApriltagPose);

    }
    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public boolean isNAN() {
        return false;
    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(xOffset, yOffset, unit);
    }

    public void recalibrate() {
        odo.recalibrateIMU();
    }

    /**
     * This returns the GoBildaPinpointDriver object used by this localizer, in case you want to
     * access any of its methods directly.
     *
     * @return returns the GoBildaPinpointDriver object used by this localizer
     */
    public GoBildaPinpointDriver getPinpoint() {
        return odo;
    }
}
