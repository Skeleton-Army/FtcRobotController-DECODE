package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.settings.SettingsOpMode;

import org.firstinspires.ftc.teamcode.enums.Alliance;

@TeleOp(name="Settings")
public class SettingsApp extends SettingsOpMode {
    @Override
    public void defineSettings() {
        add("debug_mode", "Debug Mode", new BooleanPrompt("Enable debug mode?", false));
        add("tabletop_mode", "Tabletop Mode", new BooleanPrompt("Enable tabletop mode?", false));
        add("alliance", "Select Alliance", new OptionPrompt<>("Select alliance", Alliance.RED, Alliance.BLUE));
        add("robot_centric", "Is Robot Centric", new BooleanPrompt("Is robot centric?", true));
    }
}
