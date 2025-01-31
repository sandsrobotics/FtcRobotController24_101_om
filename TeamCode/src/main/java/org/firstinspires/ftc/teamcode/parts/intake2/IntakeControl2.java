package org.firstinspires.ftc.teamcode.parts.intake2;

public class IntakeControl2 {
    public double sweeperPower;
    public int sweepLiftPosition;
    public int sweepSlidePosition;
    public int bucketLiftPosition;
    public int robotliftPosition;
    public int rotationServoDirection;
    public float strafePower;
    public int specimenServoPosition;
    public int robotLiftToZero;
    public int robotEStop;


    public IntakeControl2(double sweeperPower, int sweeperLiftPosition, int sweepSlidePosition,
                          int bucketLiftPosition, int robotLiftPosition, int rotationServoDirection,
                          float strafePower, int specimenServoPosition, int robotLiftToZero, int robotEStop) {
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.sweepSlidePosition = sweepSlidePosition;
        this.bucketLiftPosition = bucketLiftPosition;
        this.robotliftPosition = robotLiftPosition;
        this.rotationServoDirection = rotationServoDirection;
        this.strafePower = strafePower;
        this.specimenServoPosition = specimenServoPosition;
        this.robotLiftToZero = robotLiftToZero;
        this.robotEStop = robotEStop;
    }
}