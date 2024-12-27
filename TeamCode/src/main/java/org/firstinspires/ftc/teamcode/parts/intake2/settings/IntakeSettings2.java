package org.firstinspires.ftc.teamcode.parts.intake2.settings;

public class IntakeSettings2 {
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
    public final double minServoLeftSlide;
    public final double maxServoLeftSlide;
    public final double minServoRightSlide;
    public final double maxServoRightSlide;

    public IntakeSettings2(int minSlidePosition, int maxSlidePosition, int maxSlideSpeed, double tiltServoDownPosition,
                           double tiltServoUpPosition, int maxDownLiftSpeed, int maxUpLiftSpeed, int minLiftPosition,
                           int maxLiftPosition, double minRegisterVal, int tolerance, double minServoLeftSlide,
                           double maxServoLeftSlide, double minServoRightSlide, double maxServoRightSlide) {
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
        this.minServoLeftSlide = minServoLeftSlide;
        this.maxServoLeftSlide = maxServoLeftSlide;
        this.minServoRightSlide = minServoRightSlide;
        this.maxServoRightSlide = maxServoRightSlide;
    }

    public static IntakeSettings2 makeDefault(){
        return new IntakeSettings2(
                0,
                1520,
                50,
                0.85,
                0.2,
                150,
                150,
                0,
                3000,
                0.05,
                20,
                0.276,
                0.61,
                0.281,
                0.59); // in
        // max is in - min is out
    }
}
