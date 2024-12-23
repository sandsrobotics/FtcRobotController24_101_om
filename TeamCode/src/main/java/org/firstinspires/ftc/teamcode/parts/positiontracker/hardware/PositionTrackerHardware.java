package org.firstinspires.ftc.teamcode.parts.positiontracker.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import om.self.ezftc.core.Robot;

public class PositionTrackerHardware {
	public final IMU imu;
	public final IMU.Parameters parameters;

	public PositionTrackerHardware(IMU imu, IMU.Parameters parameters) {
		this.imu = imu;
		this.parameters = parameters;
	}

	public static PositionTrackerHardware makeDefault(Robot robot) {
		return makeDefault(robot, BNO055IMU.class);
	}

	public static PositionTrackerHardware makeDefault(Robot robot, Class imuType){
		IMU.Parameters parameters = new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.UP,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
		);
		return new PositionTrackerHardware((IMU)robot.opMode.hardwareMap.get(imuType,"imu"), parameters);
	}
}