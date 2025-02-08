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
    public boolean robotEStop;
    public boolean rangeEnabled;
    public int autoSupplierPosition;

    public IntakeControl2(double sweeperPower, int sweeperLiftPosition, int sweepSlidePosition,
                          int bucketLiftPosition, int robotLiftPosition, int rotationServoDirection,
                          float strafePower, int specimenServoPosition, boolean robotEStop,
                          boolean rangeEnabled, int autoSupplierPosition) {
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.sweepSlidePosition = sweepSlidePosition;
        this.bucketLiftPosition = bucketLiftPosition;
        this.robotliftPosition = robotLiftPosition;
        this.rotationServoDirection = rotationServoDirection;
        this.strafePower = strafePower;
        this.specimenServoPosition = specimenServoPosition;
        this.robotEStop = robotEStop;
        this.rangeEnabled = rangeEnabled;
        this.autoSupplierPosition = autoSupplierPosition;
    }
}