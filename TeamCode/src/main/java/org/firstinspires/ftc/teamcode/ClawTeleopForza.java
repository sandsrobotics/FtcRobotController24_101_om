package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;

@TeleOp(name="1B Claw Teleop Forza", group="Linear Opmode")
public class ClawTeleopForza extends ClawTeleopDive {
    @Override
    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeForza(robot));
    }
}