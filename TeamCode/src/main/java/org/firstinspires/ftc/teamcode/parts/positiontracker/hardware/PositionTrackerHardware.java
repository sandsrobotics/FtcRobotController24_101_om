package org.firstinspires.ftc.teamcode.parts.positiontracker.hardware;

//import com.qualcomm.hardware.bosch.BNO055IMU;
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

	public static PositionTrackerHardware makeDefault(Robot robot){
		IMU.Parameters parameters = new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.UP,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
		);
//		parameters.mode = BNO055IMU.SensorMode.IMU;
//		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//		parameters.loggingEnabled = false;
//		parameters.loggingTag = "IMU";

		return new PositionTrackerHardware(robot.opMode.hardwareMap.get(IMU.class, "imu"), parameters);
	}
}