package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "arm")
public class ArmControl extends OpMode
{
    // ServoEx is a class that expends Servo to improve developer experience
    private ServoEx vertical;// y
    private ServoEx horizontal;//x
    private  ServoEx clawPivot;
    private ServoEx clawHand;
    private Claw claw;

    private  int verticalAdjustmentScale = 1000;
    private  int HorizontalAdjustmentScale = 1000;
    private  int clawPivotAdjustmentScale = 500;
    private double minVertical = 0.05;
    private double maxVertical = 0.65;
    private double adjustmentAmout = 0.01;

     private  double minClaw = 0.1;
     private double maxClaw = 0.6;




    // Its better to use the Servo interface rather then a class
    private GamepadEx gamepad;

    // checks if right joystick was moved to either side
    private boolean wasMovedVertical()
    {
        if (gamepad.getRightY() != 0)
        {
            return true;
        }
        return false;
    }

    // checks if right joystick was moved up or down
    private Boolean wasMovedHorizontal()
    {
        if (gamepad.getRightX() != 0)
        {
            return true;
        }
        return false;

    }

    private boolean wasMovedClawPivot()
    {
        if (gamepad.getLeftX() != 0)
        {
            return true;
        }
        return false;

    }

    @Override
    public void init()
    {
        gamepad = new GamepadEx(gamepad1);
        vertical =  new ServoEx(hardwareMap, "servo1");
        horizontal = new ServoEx(hardwareMap, "servo0");
        clawPivot = new ServoEx(hardwareMap, "servo2");
        clawHand = new ServoEx(hardwareMap,  "servo3");
        GamepadEx toolOp = new GamepadEx(gamepad2);

        claw = new Claw(hardwareMap);

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> claw.clawRotateBy(0.1), claw));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> claw.clawRotateBy(-0.1), claw));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> claw.useClaw(), claw));

    }

    @Override
    public void loop()
    {

        if (wasMovedVertical())
        {
            if (vertical.get() <= maxVertical && vertical.get() >= minVertical)
            {
                vertical.set(vertical.get() - (gamepad.getRightY() / verticalAdjustmentScale));
            }
            else if (vertical.get() < minVertical) {vertical.set(minVertical);}
            else if (vertical.get() > maxVertical) {vertical.set(maxVertical);}

            //moves the arm up and down
        }

        if (wasMovedHorizontal())
        {
            if (horizontal.get() <= 0.78 && horizontal.get() >= 0.55)
            {
                horizontal.set(horizontal.get() + (gamepad.getRightX() / HorizontalAdjustmentScale));
            }
            else
            {
                if (vertical.get() < 0.4328)
                {
                    for (double i = vertical.get(); i < 0.5; i += 0.01)
                    {
                        vertical.set(i);
                    }

                }
                horizontal.set(horizontal.get() + (gamepad.getRightX() / HorizontalAdjustmentScale));
            }
        }

        if (wasMovedClawPivot())
        {
            clawPivot.set(clawPivot.get() + (gamepad.getLeftX() / clawPivotAdjustmentScale));
        }

        telemetry.addData("RightX", gamepad.getRightX());
        //prints out the joystick x

        telemetry.addData("horizontal pos", horizontal.get()); // checks the position of the servo that moves the arm horizontally

        telemetry.addData("vertical pos", vertical.get());

        if (gamepad.getButton(GamepadKeys.Button.A))
        {
            for (double i = vertical.get(); i >= minVertical; i -= adjustmentAmout)
            {
                vertical.set(i);
            }
            //vertical.set(0.05);
        }

        if (gamepad.getButton(GamepadKeys.Button.B))
        {
            for (double i = vertical.get(); i <= maxVertical; i += adjustmentAmout)
            {
               vertical.set(i);
            }
            //vertical.set(0.65);
        }
        if (gamepad.getButton(GamepadKeys.Button.X))
        {
            clawHand.set(0);
             if (clawHand.get() == 0 )
             {
                 clawHand.set(maxClaw);
             }
        }
      if (gamepad.getButton(GamepadKeys.Button.Y))
        {
            if (clawHand.get() == maxClaw)
            {
                clawHand.set(minClaw);
            }
        }
    }
}