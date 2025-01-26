package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;

@TeleOp(name="14273.2 Forza BLUE", group="14273")
public class FlipTeleopForzaBlue extends FlipTeleopForza {
    @Override
    public void extraSettings() {
        FlipbotSettings.isBlueGood = true;
        FlipbotSettings.isYellowGood = true;
        FlipbotSettings.isRedGood = false;
    }
}