package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.prompts.ValuePrompt;

import java.util.HashMap;

@TeleOp(name = "Motor Testing")
public class MotorTesting extends OpMode {

    HashMap<String, DcMotorEx> dcMotorMap;
    HashMap<String, Servo> servoMap;
    String[] motorNames;
    Prompter prompter = new Prompter(this);
    enum MotorType {
        DC_MOTOR("Dc Motor"),
        SERVO("Servo");

        // Some funny stuff to make the options in the prompter not the be the same as the Enum itself (all caps)
        private final String label;
        MotorType(final String label) {
            this.label = label;
        }
       public String getLabel() {
            return label;
       }
    }
    enum TestType {
        SIMULTANEOUS_TESTING("Simultaneous Testing"),
        INDIVIDUAL_TESTING("Individual Testing");

        // Some funny stuff to make the options in the prompter not the be the same as the Enum itself (all caps)
        private final String label;
        TestType(final String label) {
            this.label = label;
        }
        public String getLabel() {
            return label;
        }
    }

    @Override
    public void init() {
        prompter.prompt("motorType", new OptionPrompt<>("Select motor type to test", MotorType.DC_MOTOR.getLabel(), MotorType.SERVO.getLabel()))
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
                        return new OptionPrompt<>("Select test type", TestType.INDIVIDUAL_TESTING.getLabel(), TestType.SIMULTANEOUS_TESTING.getLabel());
                    }
                    return null;
                })


                .prompt("dcMotorRPM", () -> {
                    if (prompter.get("motorType") == MotorType.DC_MOTOR) {
                        return new ValuePrompt(String.format("Select the RPM at which to test Dc Motor %s at", prompter.get("motorToTest")),
                                0, 6000, 0, 10);
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
