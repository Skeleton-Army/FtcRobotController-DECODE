package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.prompts.MultiOptionPrompt;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.settings.SettingsOpMode;

@TeleOp
public class TestSettings1 extends SettingsOpMode {
    @Override
    public void defineSettings() {
        add("busssh", "buh buh", new OptionPrompt<>("bug", Buh.a, Buh.b));
        add("neea", "nana buh", new MultiOptionPrompt<>("bug", false, false, Buh.a, Buh.b));
    }
}
