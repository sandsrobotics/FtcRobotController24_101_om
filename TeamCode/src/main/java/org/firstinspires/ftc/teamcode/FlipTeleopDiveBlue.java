package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;

@TeleOp(name="2B Flipper Teleop Arcade BLUE", group="Linear Opmode")
public class FlipTeleopDiveBlue extends FlipTeleopDive {
    @Override
    public void extraSettings() {
        intake.isBlueGood = true;
        intake.isYellowGood = true;
        intake.isRedGood = false;
    }
}