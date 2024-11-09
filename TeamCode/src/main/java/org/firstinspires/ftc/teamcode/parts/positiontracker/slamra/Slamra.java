package org.firstinspires.ftc.teamcode.parts.positiontracker.slamra;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTicket;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

public class Slamra extends LoopedPartImpl<PositionTracker, SlamraSettings, ObjectUtils.Null> {
	//variables
	volatile T265Camera slamra;

	//LK ????????
	public Vector3 slamraFieldStart = null;
	public Vector3 slamraRobotOffset = new Vector3(-6.5,0,-90);
	public Vector3 slamraRawPose = new Vector3 (0,0,0);  // better name for currentPose?
	Vector3 slamraRobotPose = new Vector3 (0,0,0);
	// Position slamraRobotOffset = new Position(-6.5,0,90);
	Vector3 slamraFieldOffset = new Vector3 (0,0,0);
	public Vector3 slamraFinal = new Vector3(0,0,0);
	//End of LK ????????
	Vector3 lastPos = new Vector3();
	int timesStuck = 0;

	//constructors
	public Slamra(PositionTracker parent) {
		super(parent, "slamra");
		setSettings(SlamraSettings.makeDefault());
	}

	public Slamra(PositionTracker parent, SlamraSettings settings) {
		super(parent, "slamra");
		setSettings(settings);
	}

	@Override
	public void onBeanLoad() {

	}

	@Override
	public void onInit() {
		if (slamra == null) {
			slamra = T265Helper.getCamera(
					new T265Camera.OdometryInfo(
							getSettings().robotOffset.toPose2d(),
							getSettings().encoderCovariance
					), parent.parent.opMode.hardwareMap.appContext);
		}
		if (!slamra.isStarted()) slamra.start();
	}

	@Override
	public void onStart() {
		slamraFieldStart = parent.getCurrentPosition();
		setupFieldOffset();
	}

	@Override
	public void onSettingsUpdate(SlamraSettings settings) {
		if(slamra != null) throw new RuntimeException("slamra settings can not be updated after init");
	}

	public boolean isSlamraDead(){
		return timesStuck > 4;
	}

	@Override
	public void onRun() {
		updateSlamraPosition();
		parent.parent.opMode.telemetry.addData("slam final", slamraFinal);
		parent.parent.opMode.telemetry.addData("last pos", lastPos);
		if(!slamraFinal.equals(lastPos)) {
			parent.addPositionTicket(Slamra.class, new PositionTicket(slamraFinal));
			timesStuck = 0;
			lastPos = slamraFinal;
		}else{
			timesStuck ++;
		}

		parent.parent.opMode.telemetry.addData("slamera stuck", timesStuck);
	}

	public void updateSlamraPosition() {
		T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
//		if (up == null) return;
		Pose2d update = up.pose;
		slamraRawPose = new Vector3(update.getX(), update.getY(), Math.toDegrees(update.getHeading()));

		//LK
		updateSlamraRobotPose();
		setSlamraFinal();
	}

	public void setupFieldOffset() {
		updateSlamraPosition();
		setSlamraFieldOffset();
	}

		//LK ????????
	void updateSlamraRobotPose() {
		double sX, sY, sR, rX, rY, rR;
		sX = slamraRawPose.X;
		sY = slamraRawPose.Y;
		sR = slamraRawPose.Z;  // assuming was in radians tjk really degrees
		//rX = robotOffset.getX();
		//rY = robotOffset.getY();
		//rR = robotOffset.getHeading();         // assuming was in degrees
		rX = slamraRobotOffset.X;
		rY = slamraRobotOffset.Y;
		rR = slamraRobotOffset.Z;
		//x_robot*COS(RADIANS($C10))-y_robot*SIN(RADIANS($C10))
		slamraRobotPose = slamraRobotPose.withX(sX + (rX*Math.cos(Math.toRadians(sR)) - rY*Math.sin(Math.toRadians(sR))));
		//=x_robot*SIN(RADIANS($C10))+y_robot*COS(RADIANS($C10))
		slamraRobotPose = slamraRobotPose.withY(sY + (rX*Math.sin(Math.toRadians(sR)) + rY*Math.cos(Math.toRadians(sR))));
		slamraRobotPose = slamraRobotPose.withZ(sR + rR);

		/* Proposed slamra transformation code cleanup phase 1
		Vector3 pose1 = slamraRawPose;
		Vector3 pose2 = slamraRobotOffset;
		slamraRobotPose = new Vector3(
				pose1.X + (pose2.X*Math.cos(Math.toRadians(pose1.Z)) - pose2.Y*Math.sin(Math.toRadians(pose1.Z))),
				pose1.Y + (pose2.X*Math.sin(Math.toRadians(pose1.Z)) + pose2.Y*Math.cos(Math.toRadians(pose1.Z))),
				pose1.Z + pose2.Z
		);
		*/
	}

	// run this once at start after getting first robot pose
	void setSlamraFieldOffset() {
		double fX, fY, fR, rX, rY, rR, sR;
		if (slamraFieldStart == null) {
			fX = slamraRobotPose.X;
			fY = slamraRobotPose.Y;
			fR = slamraRobotPose.Z;
		} else {
			fX = slamraFieldStart.X;
			fY = slamraFieldStart.Y;
			fR = slamraFieldStart.Z;
		}
		rX = slamraRobotPose.X;
		rY = slamraRobotPose.Y;
		rR = slamraRobotPose.Z;
		slamraFieldOffset = slamraFieldOffset.withZ(fR - rR);
		sR = slamraFieldOffset.Z;
		//=M4*COS(RADIANS(r_field_slam))-N4*SIN(RADIANS(r_field_slam))  m4=rX, n4=rY
		slamraFieldOffset = slamraFieldOffset.withX(fX - (rX*Math.cos(Math.toRadians(sR)) - rY*Math.sin(Math.toRadians(sR))));
		//=M4*SIN(RADIANS(r_field_slam))+N4*COS(RADIANS(r_field_slam))
		slamraFieldOffset = slamraFieldOffset.withY(fY - (rX*Math.sin(Math.toRadians(sR)) + rY*Math.cos(Math.toRadians(sR))));

		/* Proposed slamra transformation code cleanup phase 1
		Vector3 sFS = slamraFieldStart;
		Vector3 sRP = slamraRobotPose;
		double offsetR = sFS.Z - sRP.Z;
		slamraFieldOffset = new Vector3 (
				sFS.X - (sRP.X*Math.cos(Math.toRadians(offsetR)) - sRP.Y*Math.sin(Math.toRadians(offsetR))),
				sFS.Y - (sRP.X*Math.sin(Math.toRadians(offsetR)) + sRP.Y*Math.cos(Math.toRadians(offsetR))),
				offsetR
		);
		*/
	}

	void setSlamraFinal() {
		//rotates slamra position to field coordinates & add offset
		double oX, oY, oR, rX, rY, rR, sR;
		rX = slamraRobotPose.X;
		rY = slamraRobotPose.Y;
		rR = slamraRobotPose.Z;
		oX = slamraFieldOffset.X;
		oY = slamraFieldOffset.Y;
		oR = slamraFieldOffset.Z;
		//=I11*COS(RADIANS(r_field_slam))-J11*SIN(RADIANS(r_field_slam))  i11=rX, j11=rY
		slamraFinal = slamraFinal.withX((rX*Math.cos(Math.toRadians(oR)) - rY*Math.sin(Math.toRadians(oR))) + oX);
		//=I11*SIN(RADIANS(r_field_slam))+J11*COS(RADIANS(r_field_slam))
		slamraFinal = slamraFinal.withY((rX*Math.sin(Math.toRadians(oR)) + rY*Math.cos(Math.toRadians(oR))) + oY);
		slamraFinal = slamraFinal.withZ(rR + oR);

		/* Proposed slamra transformation code cleanup phase 1
		Vector3 pose1 = slamraFieldOffset;
		Vector3 pose2 = slamraRobotPose;
		slamraFinal = new Vector3(
				pose1.X + (pose2.X*Math.cos(Math.toRadians(pose1.Z)) - pose2.Y*Math.sin(Math.toRadians(pose1.Z))),
				pose1.Y + (pose2.X*Math.sin(Math.toRadians(pose1.Z)) + pose2.Y*Math.cos(Math.toRadians(pose1.Z))),
				pose1.Z + pose2.Z
		);
		*/
	}
	//End of LK ????????

	@Override
	public void onStop() {
		slamra.stop();
		//slamra.free();
	}
}