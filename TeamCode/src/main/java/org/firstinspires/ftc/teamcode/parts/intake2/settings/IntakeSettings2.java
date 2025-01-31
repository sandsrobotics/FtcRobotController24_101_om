package org.firstinspires.ftc.teamcode.parts.intake2.settings;

public class IntakeSettings2 {
    public final int minSlidePosition  = 0;
    public final int maxSlidePosition = 1520;
    public final int maxSlideSpeed = 50;
    public final double tiltServoDownPosition = 0.85;
    public final double tiltServoUpPosition = 0.2;
    public final int maxDownLiftSpeed = 150;
    public final int maxUpLiftSpeed = 150;
    public final int minLiftPosition = 0;
    public final int maxLiftPosition = 2770;
    public final double minRegisterVal = 0.05;
    public final int tolerance = 20;
    public final double minServoLeftSlide = 0.559;
    public final double maxServoLeftSlide = 0.737;
    public final double minServoRightSlide = 0.559;
    public final double maxServoRightSlide = 0.737;
    public final double intakeArmAtSpecimen = 0.21;
    public final double intakeArmAtBucket = 0.64;
    public final double intakeArmStraightUp = 0.52;
    public final double intakeArmSafe = 0.48;
    public final double rotationServoMin = 0;
    public final double rotationServoMax = 1;
    public final double rotationServoStep = 0.01;
    public final double dropperServoMax = 0.36;
    public final double dropperServoMin = 0.5;
    public final double specimenServoOpenPosition = 0.485;
    public final double specimenServoClosePosition = 0.68;
    public final int specimenSafeHeight = 461;
    public final int specimenHangPosition = 1392;
    public final int specimenServoOpenHeight = 920;
    public final double parkServoPosition = 0.9;
    public final double parkServoPositionParked = 0.24;

    public IntakeSettings2() {
    }

    public static IntakeSettings2 makeDefault(){
        return new IntakeSettings2();
    }
}
