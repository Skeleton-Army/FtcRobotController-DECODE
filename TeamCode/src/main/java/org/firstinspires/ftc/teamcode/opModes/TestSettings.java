package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.prompts.BooleanPrompt;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.settings.SettingsOpMode;

@TeleOp
public class TestSettings extends SettingsOpMode {
    @Override
    public void defineSettings() {
        add("buh", "buh buh", new OptionPrompt<>("bug", Buh.a, Buh.b));
        add("na", "nana buh", new MultiOptionPrompt<>("bug", false, Buh.a, Buh.b));
    }
}
