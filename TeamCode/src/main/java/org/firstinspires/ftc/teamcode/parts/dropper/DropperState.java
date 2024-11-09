package org.firstinspires.ftc.teamcode.parts.dropper;

public class DropperState {
    public final int slidePosition;
    public final double swingAngle;
    public final double grabberPosition;

    public DropperState(int slidePosition, double swingAngle, double grabberPosition) {
        this.slidePosition = slidePosition;
        this.swingAngle = swingAngle;
        this.grabberPosition = grabberPosition;
    }
}
