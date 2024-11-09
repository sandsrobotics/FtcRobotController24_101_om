package org.firstinspires.ftc.teamcode.parts.positiontracker.slamra;

import om.self.ezftc.utils.Vector3;

public class SlamraSettings{
	////////////
	//settings//
	////////////
	public final Vector3 robotOffset;
    public final double encoderCovariance;

    public SlamraSettings(Vector3 robotOffset, double encoderCovariance) {
        this.robotOffset = robotOffset;
        this.encoderCovariance = encoderCovariance;
    }

    public static SlamraSettings makeDefault(){
        return new SlamraSettings(
                new Vector3(0,0,0),
                0.1
        );
    }
}