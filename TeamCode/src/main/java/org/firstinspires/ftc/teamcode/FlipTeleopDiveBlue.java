package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="14273.4 Arcade BLUE", group="14273")
public class FlipTeleopDiveBlue extends FlipTeleopDive {
    @Override
    public void extraSettings() {
        intake.isBlueGood = true;
        intake.isYellowGood = true;
        intake.isRedGood = false;
    }
}