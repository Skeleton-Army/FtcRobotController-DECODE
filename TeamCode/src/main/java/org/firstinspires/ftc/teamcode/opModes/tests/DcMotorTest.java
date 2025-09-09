package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
@TeleOp(name = "DcMotor Tests", group = "MotorTests")
public class DcMotorTest extends OpMode {
    /*
    ! Plan for wheel motors !

   * read all the motors from the ROBOT config

    do the warning thing on the driver hub to warn the user the pick the ROBOT up before starting the test.
    use Marrow Boolean Prompter to select one of 2 options
        - Quick
        - Full

     Quick:
        Run the wheel motor's together for 20 seconds and pull them every 2.5 seconds in record the average RPM of all the motors into an array.
        take the average of the that array, check if that average matches with the reported max RPM of the motors, (throw an error if the all the motors reported RPM don't match)
        including margins set by the user (both for above AND below the rated RPM).
        if its within spec its repot PASS else report FAIL
     Full:
         Run each motor for 20 seconds and every 2.5 seconds pull its RPM into and array.
         take the average of the that array, check if that average matches with the reported max RPM for that motor including margins set by the user (both for above AND below the rated RPM).
         if its within spec its report that motor as PASSING or FAILING

     in addition to the driver hub also report to a log file
     for and example log look at exampleLogs/example-{testType}.DcMotor.log
     */
    DcMotorEx[] motors = new DcMotorEx[4];
    MotorConfigurationType[] motorsConfig = new MotorConfigurationType[motors.length];
    @Override
    public void init() {
        /*
        ! How to get a DcMotor's Max RPM !
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "leftBack");
        MotorConfigurationType motor1Config = motor1.getMotorType();
        double motor1MaxRpm = motor1Config.getMaxRPM();
        */
    }

    @Override
    public void loop() {
    }
}
