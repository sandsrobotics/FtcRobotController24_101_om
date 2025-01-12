package org.firstinspires.ftc.teamcode.parts.intake;
public class IntakeControl {
    public int sweeperPower;
    public int sweepLiftPosition;
    public int sweepSlidePosition;
    public int bucketLiftPosition;
    public int intakePosition;

    public IntakeControl(int sweeperPower, int sweeperLiftPosition, int sweepSlidePosition, int bucketLiftPosition, int intakePosition) {
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.sweepSlidePosition = sweepSlidePosition;
        this.bucketLiftPosition = bucketLiftPosition;
        this.intakePosition = intakePosition;
    }
}