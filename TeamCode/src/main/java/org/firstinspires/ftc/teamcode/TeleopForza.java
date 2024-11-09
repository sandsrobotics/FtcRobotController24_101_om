package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;
@Disabled
@TeleOp(name="2 Teleop Forza", group="Linear Opmode")
public class TeleopForza extends TestPixel {
    @Override
    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeForza(robot));
    }
}