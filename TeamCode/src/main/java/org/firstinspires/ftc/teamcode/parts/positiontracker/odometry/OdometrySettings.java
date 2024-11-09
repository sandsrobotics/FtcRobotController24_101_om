package org.firstinspires.ftc.teamcode.parts.positiontracker.odometry;

import om.self.ezftc.utils.Vector3;

public class OdometrySettings {
    public final double ticksPerInch;
    public final double ticksPerRotation;
    public final Vector3 robotOffset;

    public final double leftYServoUp;
    public final double leftYServoDown;
    public final double rightYServoUp;
    public final double rightYServoDown;
    public final double XServoUp;
    public final double XServoDown;

    public OdometrySettings(double ticksPerInch, double ticksPerRotation, Vector3 robotOffset, double leftYServoUp, double leftYServoDown, double rightYServoUp, double rightYServoDown, double XServoUp, double XServoDown) {
        this.ticksPerInch = ticksPerInch;
        this.ticksPerRotation = ticksPerRotation;
        this.robotOffset = robotOffset;
        this.leftYServoUp = leftYServoUp;
        this.leftYServoDown = leftYServoDown;
        this.rightYServoUp = rightYServoUp;
        this.rightYServoDown = rightYServoDown;
        this.XServoUp = XServoUp;
        this.XServoDown = XServoDown;
    }

    //76335 expected ticks (*0.343)
    public static OdometrySettings makeForOdoBot(){
        return new OdometrySettings(
                (((40268+40341)/2+(40301+40343)/2+(40275+40358)/2))/(117.5*3), //82300 / 48.0, old: 220,000
                ((42425+41303)+(41873+41900)+(41394+42559)+(41947+41794))/16, //169619 - old: 440928
                new Vector3(0, 1.625,0), // old offset : -.25, 1.75, 0. yWheel offsets: x - 9.75/2, y - 3
                .57,
                1,
                .485,
                .08,
                .18,
                .75
        );
    }
}
//                new Vector3(-0.25, 1.5,0), // old offset : -.25, 1.75, 0. yWheel offsets: x - 9.75/2, y - 3