package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="14273.2 Forza BLUE", group="14273")
public class FlipTeleopForzaBlue extends FlipTeleopForza {
    @Override
    public void extraSettings() {
        intake.isBlueGood = true;
        intake.isYellowGood = true;
        intake.isRedGood = false;
    }
}