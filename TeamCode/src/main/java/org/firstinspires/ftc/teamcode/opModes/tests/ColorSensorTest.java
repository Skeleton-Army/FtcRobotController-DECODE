package org.firstinspires.ftc.teamcode.opModes.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class ColorSensorTest extends OpMode {
    private NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];

        int r = (int) (colors.red * 255);
        int g = (int) (colors.green * 255);
        int b = (int) (colors.blue * 255);

        Color.RGBToHSV(
                r,
                g,
                b,
                hsv
        );

        float hue = hsv[0];

        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Is Green", hue >= 79 && hue <= 155);
        telemetry.addData("Is Purple", hue >= 273 && hue <= 321);
        telemetry.update();
    }
}
