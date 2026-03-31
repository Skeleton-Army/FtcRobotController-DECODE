package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@TeleOp(name = "arm")
public class ArmControl extends OpMode
{
    // ServoEx is a class that expends Servo to improve developer experience
    private ServoEx vertical;//;p
    private ServoEx horizontal;

    // Its better to use the Servo interface rather then a class
    private GamepadEx gamepad;

    // checks if right joystick was moved to either side
    private boolean wasMovedVertical()
    {
        if (gamepad.getRightX() != 0)
        {
            return true;
        }
        return false;
    }

    // checks if right joystick was moved up or down
    private boolean wasMovedHorizontal()
    {
        if (gamepad.getRightY() != 0)
        {
            return true;
        }
        return false;

    }

    @Override
    public void init()
    {
        gamepad = new GamepadEx(gamepad1);
        vertical =  new ServoEx(hardwareMap, "servo0");
        horizontal = new ServoEx(hardwareMap, "servo1");

    }

    @Override
    public void loop()
    {
        if (wasMovedVertical())
        {
            vertical.set(vertical.get() + (gamepad.getRightX() / 1000));
            //moves the arm to the sides
        }

        if (wasMovedHorizontal())
        {
            horizontal.set(horizontal.get() + (gamepad.getRightY() / 20000));
            //moves the arm up and down
        }

        telemetry.addData("RightX", gamepad.getRightX());
        //prints out the joystick x

        telemetry.addData("horizontal pos", horizontal.get());

        if (gamepad.gamepad.x)
        {
            horizontal.set(0);
        }

        if (gamepad.gamepad.circle)
        {
            horizontal.set(1);
        }

    }
}
