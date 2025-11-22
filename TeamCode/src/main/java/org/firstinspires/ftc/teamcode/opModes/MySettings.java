package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.settings.SettingsOpMode;

@TeleOp
public class MySettings extends SettingsOpMode {
    @Override
    public void defineSettings() {
        add("debug", "Debug Mode", new BooleanPrompt("Enable debug mode", false));
        add("tabletop", "Tabletop Mode", new BooleanPrompt("Enable tabletop mode (disables movement)", false));
        add("alliance", "Select Alliance", new OptionPrompt<>("Select alliance", Alliance.RED, Alliance.BLUE));
    }
}
