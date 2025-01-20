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
    public final double intakeArmAtSpecimen;
    public final double intakeArmAtBucket;
    public final double intakeArmStraightUp;
    public final double intakeArmSafe;
    public final double rotationServoMin;
    public final double rotationServoMax;
    public final double rotationServoStep;
    public final double dropperServoMax;
    public final double dropperServoMin;
    public final double specimenServoOpenPosition;
    public final double specimenServoClosePosition;
    public final int specimenSafeHeight;
    public final int specimenHangPosition;
    public final int specimenServoOpenHeight;


    public IntakeSettings2(int minSlidePosition, int maxSlidePosition, int maxSlideSpeed, double tiltServoDownPosition,
                           double tiltServoUpPosition, int maxDownLiftSpeed, int maxUpLiftSpeed, int minLiftPosition,
                           int maxLiftPosition, double minRegisterVal, int tolerance, double minServoLeftSlide,
                           double maxServoLeftSlide, double minServoRightSlide, double maxServoRightSlide,
                           double intakeArmAtSpecimen, double intakeArmAtBucket, double intakeArmStraightUp, double intakeArmSafe,
                           double rotationServoMin, double rotationServoMax, double rotationServoStep, double dropperServoMax, double dropperServoMin,
                           double specimenServoOpenPosition, double specimenServoClosePosition, int specimenSafeHeight,
                           int specimenHangPosition, int specimenServoOpenHeight) {
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
        this.intakeArmAtSpecimen = intakeArmAtSpecimen;
        this.intakeArmAtBucket = intakeArmAtBucket;
        this.intakeArmStraightUp = intakeArmStraightUp;
        this.intakeArmSafe = intakeArmSafe;
        this.rotationServoMin = rotationServoMin;
        this.rotationServoMax = rotationServoMax;
        this.rotationServoStep = rotationServoStep;
        this.dropperServoMax = dropperServoMax;
        this.dropperServoMin = dropperServoMin;
        this.specimenServoOpenPosition = specimenServoOpenPosition;
        this.specimenServoClosePosition = specimenServoClosePosition;
        this.specimenSafeHeight = specimenSafeHeight;
        this.specimenHangPosition = specimenHangPosition;
        this.specimenServoOpenHeight = specimenServoOpenHeight;
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
                2770,
                0.05,
                20,
                0.559,
                0.737,
                0.559,
                0.737,
                0.21,
                0.64,
                0.52,
                0.48,
                0,
                1,
                0.01,
                0.2,
                0.5,
                0.485,
                0.68,
                461,
                1392,
                920
        );
    }
}
