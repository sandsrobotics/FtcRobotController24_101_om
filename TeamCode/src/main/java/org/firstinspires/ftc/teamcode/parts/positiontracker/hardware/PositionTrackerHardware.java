package org.firstinspires.ftc.teamcode.parts.positiontracker.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import om.self.ezftc.core.Robot;

public class PositionTrackerHardware {
    public final IMU imu;
    public final IMU.Parameters parameters;

    public PositionTrackerHardware(IMU imu, IMU.Parameters parameters) {
        this.imu = imu;
        this.parameters = parameters;
    }

    public static PositionTrackerHardware makeDefault(Robot robot){
//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                )
//        );
        return new PositionTrackerHardware(null, null);
    }
}