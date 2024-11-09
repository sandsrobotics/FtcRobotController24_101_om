package org.firstinspires.ftc.teamcode.parts.positiontracker.settings;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;

public class PositionTrackerSettings {

	public final AxesOrder axesOrder;
	public final boolean flipAngle;
	public final int stalePosTime;
	public final Vector3 positionTolerance;
	public final Vector3 startPosition;

	public PositionTrackerSettings(AxesOrder axesOrder, boolean flipAngle, int stalePosTime, Vector3 positionTolerance, Vector3 startPosition) {
		this.axesOrder = axesOrder;
		this.flipAngle = flipAngle;
		this.stalePosTime = stalePosTime;
		this.positionTolerance = positionTolerance;
		this.startPosition = startPosition;
	}

	public static PositionTrackerSettings makeDefault(){
		return new PositionTrackerSettings(
				AxesOrder.XYZ,
				false,
				100,
				new Vector3(5,5,10),
				new Vector3(-1.5 * Constants.tileSide,62,90)
		);
	}

	public PositionTrackerSettings withPosition(Vector3 position){
		return new PositionTrackerSettings(axesOrder, flipAngle, stalePosTime, positionTolerance, position);
	}
}