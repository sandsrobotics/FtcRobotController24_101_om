package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;
import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;

@TeleOp(name="14273.0 DEMO", group="14273")
public class FlipTeleopDemo extends FlipTeleopDive {

    @Override
    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeDemo1(robot));
    }

    @Override
    public void extraSettings() {
        FlipbotSettings.isBlueGood = true;
        FlipbotSettings.isYellowGood = true;
        FlipbotSettings.isRedGood = false;
        FlipbotSettings.isDemoMode = true;
    }
}