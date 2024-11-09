package org.firstinspires.ftc.teamcode.parts.intake.settings;

public class IntakeSettings {
    public final int minSlidePosition;
    public final int maxSlidePosition;
    public final int maxDownSlideSpeed;
    public final int maxUpSlideSpeed;
    public final double minRegisterVal;
    public final double tolerance;
    public final double sweepLiftServoStorePosition;
    public final double sweepLiftServoDownPosition;
    public final double sweepLiftServoStackPosition;
    public final int maxDownLiftSpeed;
    public final int maxUpLiftSpeed;
    public final int maxLiftPosition;
    public final int minLiftPosition;
    public final double grabberOpenPosition;
    public final double grabberGripTwoPosition;
    public final double grabberGripOnePosition;
    public final double swingLeftSafePosition;
    public final double swingRightSafePosition;
    public final double swingLeftDropPosition;
    public final double swingRightDropPosition;
    public final int maxPix;
    public final int maxPixLine;
    public final double launchAngleStorePosition;
    public final double launchAngleArmPosition;
    public final double launchReleaseLockPosition;
    public final double launchReleaseUnlockPosition;
    public final double grabberPrimePosition;
    public final double swingLeftMaxPosition;
    public final double sweepLiftServoStackTopPosition;
    public final double swingLeftActualMaxPosition;
    public final double launcherLoadPosition;



    public IntakeSettings(int minSlidePosition, int maxSlidePosition, int maxDownSlideSpeed, int maxUpSlideSpeed, double minRegisterVal, int maxUpLiftSpeed,
                          int maxDownLiftSpeed, double tolerance, double sweepLiftServoDownPosition, double sweepLiftServoStackPosition,
                          double sweepLiftServoStorePosition, int maxLiftPosition, int minLiftPosition, double grabberOpenPosition, double grabberGripTwoPosition,
                          double grabberGripOnePosition, double swingLeftSafePosition, double swingRightSafePosition, double swingLeftDropPosition, double swingRightDropPosition, int maxPix, int maxPixLine, double launchAngleStorePosition,
                          double launchAngleArmPosition, double launchReleaseLockPosition, double launchReleaseUnlockPosition, double grabberPrimePosition, double swingLeftMaxPosition, double sweepLiftServoStackTopPosition, double swingLeftActualMaxPosition,
                          double launcherLoadPosition) {
        this.minSlidePosition = minSlidePosition;
        this.maxSlidePosition = maxSlidePosition;
        this.maxDownSlideSpeed = maxDownSlideSpeed;
        this.maxUpSlideSpeed = maxUpSlideSpeed;
        this.minRegisterVal = minRegisterVal;
        this.maxDownLiftSpeed = maxDownLiftSpeed;
        this.maxUpLiftSpeed = maxUpLiftSpeed;
        this.tolerance = tolerance;
        this.sweepLiftServoStorePosition = sweepLiftServoStorePosition;
        this.sweepLiftServoDownPosition = sweepLiftServoDownPosition;
        this.sweepLiftServoStackPosition = sweepLiftServoStackPosition;
        this.maxLiftPosition = maxLiftPosition;
        this.minLiftPosition = minLiftPosition;
        this.grabberOpenPosition = grabberOpenPosition;
        this.grabberGripTwoPosition = grabberGripTwoPosition;
        this.grabberGripOnePosition = grabberGripOnePosition;
        this.swingLeftSafePosition = swingLeftSafePosition;
        this.swingRightSafePosition = swingRightSafePosition;
        this.swingLeftDropPosition = swingLeftDropPosition;
        this.swingRightDropPosition = swingRightDropPosition;
        this.maxPix = maxPix;
        this.maxPixLine = maxPixLine;
        this.launchAngleStorePosition = launchAngleStorePosition;
        this.launchAngleArmPosition = launchAngleArmPosition;
        this.launchReleaseLockPosition = launchReleaseLockPosition;
        this.launchReleaseUnlockPosition = launchReleaseUnlockPosition;
        this.grabberPrimePosition = grabberPrimePosition;
        this.swingLeftMaxPosition = swingLeftMaxPosition;
        this.sweepLiftServoStackTopPosition = sweepLiftServoStackTopPosition;
        this.swingLeftActualMaxPosition = swingLeftActualMaxPosition;
        this.launcherLoadPosition = launcherLoadPosition;
    }

    // left safe: ,.14 drop: .5 FOR ACTUAL MAX, .6
    // right safe: .87, drop: .48

    public static IntakeSettings makeDefault(){
        return new IntakeSettings(
                0,
                3350,
                300,
                300,
                0.05,
                300,
                300,
                2,
                0.209,
                0.320,
                0.8,
                8699,
                201,
                .95,
                .18,
                .585,
                .168,
                .842,
                .529,
                .48,
                12,
                7,
                .46,
                .59,
                .660,
                .325,
                .42,
                .56,
                .347,
                .6,
                .58
                );
//        swingLeftSafePosition Gobuilda .158
//        swingRightSafePosition Goubilda .842
                //  old launchReleaseLockPosition  .482,
                //  old launchReleaseUnlockPosition .702
               //new Launcher load position .58
    }
}
