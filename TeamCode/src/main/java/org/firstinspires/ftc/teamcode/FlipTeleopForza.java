package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;

@TeleOp(name="1A Flip Teleop Forza RED", group="Linear Opmode")
public class FlipTeleopForza extends FlipTeleopDive {
    @Override
    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeForza1(robot));
    }
}