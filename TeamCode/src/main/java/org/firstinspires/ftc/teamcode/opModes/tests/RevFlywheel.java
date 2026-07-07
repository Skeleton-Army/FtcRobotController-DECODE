package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL1_INVERTED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL1_NAME;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL2_INVERTED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL2_NAME;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.FLYWHEEL_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorEx;
import org.firstinspires.ftc.teamcode.utilities.ModifiedMotorGroup;

@TeleOp()
public class RevFlywheel extends OpMode {
    private ModifiedMotorEx flywheel1;
    private ModifiedMotorEx flywheel2;
    private ModifiedMotorGroup flywheel;

    @Override
    public void init() {
        flywheel1 = new ModifiedMotorEx(hardwareMap, FLYWHEEL1_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
        flywheel1.setInverted(FLYWHEEL1_INVERTED);

        flywheel2 = new ModifiedMotorEx(hardwareMap, FLYWHEEL2_NAME,
                FLYWHEEL_MOTOR.getCPR() * FLYWHEEL_GEAR_RATIO,
                FLYWHEEL_MOTOR.getRPM() / FLYWHEEL_GEAR_RATIO);
        flywheel2.setInverted(FLYWHEEL2_INVERTED);

        flywheel = new ModifiedMotorGroup(flywheel1, flywheel2);
    }

    @Override
    public void loop() {
        flywheel.set(gamepad1.right_trigger);
    }
}
