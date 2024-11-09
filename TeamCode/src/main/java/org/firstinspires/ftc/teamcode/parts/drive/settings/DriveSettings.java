package org.firstinspires.ftc.teamcode.parts.drive.settings;

import om.self.ezftc.utils.Vector3;

public class DriveSettings {
    //other
    public final DriveMode driveMode;
    public final Vector3 smoothingValues;
    public final boolean useSmoothing;

    public DriveSettings(DriveMode driveMode, Vector3 smoothingValues, boolean useSmoothing) {
        this.driveMode = driveMode;
        this.smoothingValues = smoothingValues;
        this.useSmoothing = useSmoothing;
    }

    public static DriveSettings makeDefault(){
        return new DriveSettings(
                DriveSettings.DriveMode.MECANUM,
                new Vector3(0.075,0.075,0.075),
                true
        );
    }

    public enum DriveMode{
        TANK,
        MECANUM,
        OMNI
    }
}
