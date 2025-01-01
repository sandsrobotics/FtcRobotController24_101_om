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
    public final double intakeArmMin;
    public final double intakeArmMax;
    public final double intakeArmDefault;
    public final double rotationServoMin;
    public final double rotationServoMax;
    public final double rotationServoStep;

    public IntakeSettings2(int minSlidePosition, int maxSlidePosition, int maxSlideSpeed, double tiltServoDownPosition,
                           double tiltServoUpPosition, int maxDownLiftSpeed, int maxUpLiftSpeed, int minLiftPosition,
                           int maxLiftPosition, double minRegisterVal, int tolerance, double minServoLeftSlide,
                           double maxServoLeftSlide, double minServoRightSlide, double maxServoRightSlide,
                           double intakeArmMin, double intakeArmMax, double intakeArmDefault,
                           double rotationServoMin, double rotationServoMax, double rotationServoStep) {
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
        this.intakeArmMin = intakeArmMin;
        this.intakeArmMax = intakeArmMax;
        this.intakeArmDefault = intakeArmDefault;
        this.rotationServoMin = rotationServoMin;
        this.rotationServoMax = rotationServoMax;
        this.rotationServoStep = rotationServoStep;
    }

    public static IntakeSettings2 makeDefault() {
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
                0.470,
                0.778,
                0.470,
                0.778,
                0.21,
                0.62,
                0.52,
                0,
                1,
                0.01 // Step increment for rotation servo //0.21
                //minright;.281 minleft;.276
        );
    }
}
