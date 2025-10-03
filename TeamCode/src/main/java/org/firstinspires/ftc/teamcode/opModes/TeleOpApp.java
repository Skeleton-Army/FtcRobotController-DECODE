package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Shooter shooter;

    Motor intake;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        //shooter = new Shooter(hardwareMap, follower.poseTracker);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        /*new Trigger(() -> gamepad1.right_trigger > 0.1)
                .whenActive(new ShootCommand(shooter));
*/

        intake = new Motor(hardwareMap, "intake");
        schedule(
                // TODO: Set shooter angle to GOAL
        );
    }

    @Override
    public void run() {
        // Tuning.drawCurrent();
        super.run();
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        gamepadEx1.readButtons();
        if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
        {
            intake.set(-0.75);
        }

        if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            intake.set(-1);
        }

        else {
            intake.set(0);
        }
        telemetry.addData("right bumper: ", gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER));
        telemetry.update();
    }
}
