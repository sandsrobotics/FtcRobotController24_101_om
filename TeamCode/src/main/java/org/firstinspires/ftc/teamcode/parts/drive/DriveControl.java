package org.firstinspires.ftc.teamcode.parts.drive;

import om.self.ezftc.utils.Vector3;

public class DriveControl {
    public Vector3 power;
    public boolean stop;

    public DriveControl() {
    }

    public DriveControl(Vector3 power, boolean stop) {
        this.power = power;
        this.stop = stop;
    }
}
