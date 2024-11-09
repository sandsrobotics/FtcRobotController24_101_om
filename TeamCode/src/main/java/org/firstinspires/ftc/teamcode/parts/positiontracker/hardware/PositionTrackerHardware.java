package org.firstinspires.ftc.teamcode.parts.positiontracker.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import om.self.ezftc.core.Robot;

public class PositionTrackerHardware {
	public final BNO055IMU imu;
	public final BNO055IMU.Parameters parameters;

	public PositionTrackerHardware(BNO055IMU imu, BNO055IMU.Parameters parameters) {
		this.imu = imu;
		this.parameters = parameters;
	}

	public static PositionTrackerHardware makeDefault(Robot robot){
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
		parameters.loggingEnabled = false;
		parameters.loggingTag = "IMU";

		return new PositionTrackerHardware(robot.opMode.hardwareMap.get(BNO055IMU.class, "imu"), parameters);
	}
}