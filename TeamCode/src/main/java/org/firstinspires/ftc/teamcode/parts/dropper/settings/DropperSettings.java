package org.firstinspires.ftc.teamcode.parts.dropper.settings;

public class DropperSettings {
    public final int minSlidePosition;
    public final int maxSlidePosition;
    public final int maxDownSlideSpeed;
    public final int maxUpSlideSpeed;
    public final double minRegisterVal;
    public final double tolerance;
    public final double grabberOpenPosition;
    public final double grabberGripTwoPosition;
    public final double grabberGripOnePosition;
    public final double swingLeftSafePosition;
    public final double swingRightSafePosition;
    public final double swingLeftDropPosition;
    public final double swingRightDropPosition;
    public final int maxPix;
    public final int maxPixLine;
    public final double grabberPrimePosition;
    public final double swingLeftMaxPosition;
    public final double swingLeftActualMaxPosition;

    public DropperSettings(int minSlidePosition, int maxSlidePosition, int maxDownSlideSpeed, int maxUpSlideSpeed, double minRegisterVal, double tolerance, double grabberOpenPosition, double grabberGripTwoPosition, double grabberGripOnePosition, double swingLeftSafePosition, double swingRightSafePosition, double swingLeftDropPosition, double swingRightDropPosition, int maxPix, int maxPixLine, double grabberPrimePosition, double swingLeftMaxPosition, double swingLeftActualMaxPosition) {
        this.minSlidePosition = minSlidePosition;
        this.maxSlidePosition = maxSlidePosition;
        this.maxDownSlideSpeed = maxDownSlideSpeed;
        this.maxUpSlideSpeed = maxUpSlideSpeed;
        this.minRegisterVal = minRegisterVal;
        this.tolerance = tolerance;
        this.grabberOpenPosition = grabberOpenPosition;
        this.grabberGripTwoPosition = grabberGripTwoPosition;
        this.grabberGripOnePosition = grabberGripOnePosition;
        this.swingLeftSafePosition = swingLeftSafePosition;
        this.swingRightSafePosition = swingRightSafePosition;
        this.swingLeftDropPosition = swingLeftDropPosition;
        this.swingRightDropPosition = swingRightDropPosition;
        this.maxPix = maxPix;
        this.maxPixLine = maxPixLine;
        this.grabberPrimePosition = grabberPrimePosition;
        this.swingLeftMaxPosition = swingLeftMaxPosition;
        this.swingLeftActualMaxPosition = swingLeftActualMaxPosition;
    }
    // left safe: ,.14 drop: .5 FOR ACTUAL MAX, .6
    // right safe: .87, drop: .48

    public static DropperSettings makeDefault(){
        return new DropperSettings(
                0,
                3350,
                300,
                300,
                0.05,
                2,
                .95,
                .15,
                .56,
                .158,
                .842,
                .52,
                .48,
                12,
                7,
                .42,
                .56,
                .6
        );
    }

}
