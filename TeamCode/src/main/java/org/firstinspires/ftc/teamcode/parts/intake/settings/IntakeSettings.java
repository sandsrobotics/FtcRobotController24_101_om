package org.firstinspires.ftc.teamcode.parts.intake.settings;

public class IntakeSettings {
    public final int minSlidePosition;
    public final int maxSlidePosition;
    public final int maxSlideSpeed;
    public final double tiltServoDownPosition;
    public final double tiltServoUpPosition;
    public final int maxDownLiftSpeed;
    public final int maxUpLiftSpeed;
    public final int minLiftPosition;
    public final int maxLiftPosition;
    public final double minRegisterVal;
    public final int tolerance;

    public IntakeSettings(int minSlidePosition, int maxSlidePosition, int maxSlideSpeed, double tiltServoDownPosition,
                          int maxDownLiftSpeed, int maxUpLiftSpeed, int minLiftPosition, int maxLiftPosition,
                          double tiltServoUpPosition, double minRegisterVal, int tolerance) {
        this.minSlidePosition = minSlidePosition;
        this.maxSlidePosition = maxSlidePosition;
        this.maxSlideSpeed = maxSlideSpeed;
        this.tiltServoDownPosition = tiltServoDownPosition;
        this.tiltServoUpPosition = tiltServoUpPosition;
        this.minRegisterVal = minRegisterVal;
        this.tolerance = tolerance;
        this.maxDownLiftSpeed = maxDownLiftSpeed;
        this.maxUpLiftSpeed = maxUpLiftSpeed;
        this.minLiftPosition = minLiftPosition;
        this.maxLiftPosition = maxLiftPosition;
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings(
                0,
                1520,
                50,
                0.9,
                150,
                150,
                0,
                3000,
                0,
                0.05,
                20);
    }
}
