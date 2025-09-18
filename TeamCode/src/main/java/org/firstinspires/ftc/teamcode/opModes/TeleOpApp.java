package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class TeleOpApp extends CommandOpMode {
    private Follower follower;
    private Shooter shooter;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);

        shooter = new Shooter(hardwareMap, follower.poseTracker);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        new Trigger(() -> gamepad1.right_trigger > 0.1)
                .whenActive(new ShootCommand(shooter));

        schedule(
                // TODO: Set shooter angle to GOAL
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
    }
}
