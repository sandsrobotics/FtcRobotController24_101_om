package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;

@TeleOp(name="1B Flip Teleop Forza BLUE", group="Linear Opmode")
public class FlipTeleopForzaBlue extends FlipTeleopForza {
    @Override
    public void extraSettings() {
        intake.isBlueGood = true;
        intake.isYellowGood = true;
        intake.isRedGood = false;
    }
}