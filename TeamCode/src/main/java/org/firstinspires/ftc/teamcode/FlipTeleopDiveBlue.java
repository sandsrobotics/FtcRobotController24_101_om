package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;

@TeleOp(name="14273.4 Arcade BLUE", group="B14273")
public class FlipTeleopDiveBlue extends FlipTeleopDive {
    @Override
    public void extraSettings() {
        FlipbotSettings.isBlueGood = true;
        FlipbotSettings.isYellowGood = true;
        FlipbotSettings.isRedGood = false;
    }
}