package org.firstinspires.ftc.teamcode.depricated.lifter;

public class LifterControl {
    public double lifterPower;
    public double turningPower;
    public boolean close;
    public boolean forceMoveLifter;

    public static int flipOpen; //TODO make non static

    public LifterControl(double lifterPower, double turningPower, boolean close) {
        this.lifterPower = lifterPower;
        this.turningPower = turningPower;
        this.close = close;
    }

    public LifterControl(double lifterPower, double turningPower, boolean close, boolean forceMoveLifter) {
        this.lifterPower = lifterPower;
        this.turningPower = turningPower;
        this.close = close;
        this.forceMoveLifter = forceMoveLifter;
    }
}
