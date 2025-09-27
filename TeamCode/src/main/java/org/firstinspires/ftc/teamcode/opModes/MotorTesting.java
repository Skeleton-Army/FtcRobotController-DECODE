package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import java.util.HashMap;

@TeleOp(name = "Motor Testing")
public class MotorTesting extends OpMode {

    HashMap<String, DcMotorEx> dcMotorMap;
    HashMap<String, Servo> servoMap;
    String[] motorNames;
    Prompter prompter = new Prompter(this);
    enum MotorType {
        DC_MOTOR,
        SERVO
    }
    enum TestType {
        SIMULTANEOUS_TESTING,
        INDIVIDUAL_TESTING
    }

    @Override
    public void init() {
        prompter.prompt("motorType", new OptionPrompt<>("Select motor type to test", MotorType.DC_MOTOR, MotorType.SERVO))
                // TODO: once Multiple Option prompt is out replace this to be able to select multiple motors
                .prompt("motorToTest", () -> {
                    final MotorType selectedMotor = prompter.get("motorType");
                   if ( selectedMotor == MotorType.DC_MOTOR) {
                        dcMotorMap = initAllDevices(DcMotorEx.class);
                        motorNames = dcMotorMap.keySet().toArray(new String[0]);
                        return new OptionPrompt<>("Select motor to run", motorNames);
                   } else if (selectedMotor == MotorType.SERVO) {
                       servoMap = initAllDevices(Servo.class);
                       motorNames = servoMap.keySet().toArray(new String[0]);
                       return new OptionPrompt<>("Select servo to run", motorNames);
                   }
                    return null;
                })
                .prompt("testType", () -> {
                    if (prompter.get("motorType") == MotorType.DC_MOTOR) {
                        return new OptionPrompt<>("Select test type", TestType.INDIVIDUAL_TESTING, TestType.SIMULTANEOUS_TESTING);
                    }
                    return null;
                })

                /*
                TODO: pretty sure I can regex HardwareDevice.getConnectionInfo() and HardwareDevice.getManufacture() and compare that against a constants file
                 I think a Json file would be really nice when adding new motors and would be a lot nicer to read then a MotorConstants.java
                 which would be easier to implement then the Json file
                 so the prompt below will no be needed once it is implemented and make not hard coded and just less spaghetti
                */
                .prompt("dcMotorRPM", () -> {
                    if (prompter.get("motorType") == MotorType.DC_MOTOR) {
                        return new OptionPrompt<>("Select the max RPM of motor: " + prompter.get("motorToTest"),
                                "6,000", "1,620", "1,150", "435", "312", "223", "117", "100", "84", "60", "43", "30");
                    }
                    return null;
                })
                .prompt("startTest", () -> {
                    if (prompter.get("motorType") == MotorType.DC_MOTOR) {
                        return new BooleanPrompt(String.format("Start test for motor %s at %s RPM?",
                                prompter.get("motorToTest"), prompter.get("dcMotorRPM")), true);
                    } else if (prompter.get("motorType") == MotorType.SERVO) {
                        return new BooleanPrompt(String.format("Start test for motor %s?", prompter.get("motorToTest")), true);
                    }
                    return null;
                })
                .onComplete(this::startTest);

    }
    @Override
    public void init_loop() {
        prompter.run();
    }

    @Override
    public void loop() {

    }

    public <T extends HardwareDevice> HashMap<String, T> initAllDevices(Class<T> type) {
        HashMap<String, T> motorMap = new HashMap<>();
        for (T motor : hardwareMap.getAll(type)) {
            motorMap.put(hardwareMap.getNamesOf(motor).toArray()[0].toString(), motor);
        }
        return motorMap;
    }

    private void startTest() {
        MotorType motorType;
        int motorMaxRPM;
        String motorName;

        if (prompter.get("startTest").equals(false)) {
            telemetry.addLine("Stopping opMode");
            requestOpModeStop();
        }

        motorType = prompter.get("motorType");
        motorName = prompter.get("motorToTest");
        if (motorType == MotorType.SERVO) {
            // run Servo test
        }
        motorMaxRPM = Integer.parseInt(prompter.get("dcMotorRPM"));

        if (prompter.get("testType") == TestType.SIMULTANEOUS_TESTING) {
            // run Simultaneous motor testing
        } else {
            // run Individual motor testing
        }
    }
}
