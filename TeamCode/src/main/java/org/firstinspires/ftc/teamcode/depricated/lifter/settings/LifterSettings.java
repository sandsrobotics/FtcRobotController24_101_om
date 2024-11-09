package org.firstinspires.ftc.teamcode.depricated.lifter.settings;

public class LifterSettings {
    //turn servos
    public final double turnServoMinPosition;
    public final double turnServoMaxPosition;
    public final double turnServoStartPosition;
    public final double rightTurnServoOffset; //TODO return to final
    //grabber servo
    public final double grabberServoOpenPos;
    public final double grabberServoWideOpenPos;
    public final double grabberServoClosePos;
    //motor
    public final double minRegisterVal;
    public final int maxDownLiftSpeed;
    public final int maxUpLiftSpeed;
    public final int minLiftPosition;
    public final int maxLiftPosition;
    public final int tolerance;

    public final int maxCones;
    public final int maxPoles;


    public LifterSettings(double turnServoMinPosition, double turnServoMaxPosition, double turnServoStartPosition, double rightTurnServoOffset, double grabberServoOpenPos, double grabberServoWideOpenPos, double grabberServoClosePos, double minRegisterVal, int maxDownLiftSpeed, int maxUpLiftSpeed, int minLiftPosition, int maxLiftPosition, int tolerance, int maxCones, int maxPoles) {
        this.turnServoMinPosition = turnServoMinPosition;
        this.turnServoMaxPosition = turnServoMaxPosition;
        this.turnServoStartPosition = turnServoStartPosition;
        this.rightTurnServoOffset = rightTurnServoOffset;
        this.grabberServoOpenPos = grabberServoOpenPos;
        this.grabberServoWideOpenPos = grabberServoWideOpenPos;
        this.grabberServoClosePos = grabberServoClosePos;
        this.minRegisterVal = minRegisterVal;
        this.maxDownLiftSpeed = maxDownLiftSpeed;
        this.maxUpLiftSpeed = maxUpLiftSpeed;
        this.minLiftPosition = minLiftPosition;
        this.maxLiftPosition = maxLiftPosition;
        this.tolerance = tolerance;
        this.maxCones = maxCones;
        this.maxPoles = maxPoles;
    }

    public static LifterSettings makeDefault(){
        return new LifterSettings(
                0.064,
                0.93,
                0.93,
                0,
                .7,
                0.67,
                0.82,
                0.05,
                150,
                150,
                0,
                3000,
                20,
                4,
                3
        );
    }
}
