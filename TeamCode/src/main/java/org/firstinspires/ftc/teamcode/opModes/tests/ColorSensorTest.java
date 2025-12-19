package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends OpMode {
    private SensorRevColorV3 sensor;

    @Override
    public void init() {
        sensor = new SensorRevColorV3(hardwareMap, "colorSensor", DistanceUnit.CM);
    }

    @Override
    public void loop() {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = new float[3];
        sensor.RGBtoHSV(r, g, b, hsv);

        float hue = hsv[0];
        double distance = sensor.distance();

        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);

        telemetry.addData("Hue", hue);

        telemetry.addData("Is Green", hue >= 79 && hue <= 155);
        telemetry.addData("Is Purple", hue >= 273 && hue <= 321);

        telemetry.addData("Distance (" + DistanceUnit.CM + ")", "%.1f", distance);

        telemetry.update();
    }
}
