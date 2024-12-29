package org.firstinspires.ftc.teamcode.parts.intake2;

public class IntakeControl2 {
    public int sweeperPower;
    public int sweepLiftPosition;
    public int sweepSlidePosition;
    public int bucketLiftPosition;
    public int robotliftPosition;
    public int robotlift0Position;
    public int robotlifthangPosition;
    public int rotationServoDirection;

    public IntakeControl2(int sweeperPower, int sweeperLiftPosition, int sweepSlidePosition,
                          int bucketLiftPosition, int robotLiftPosition, int robotLift0Position,
                          int robotLifthangPosition, int rotationServoDirection) {
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.sweepSlidePosition = sweepSlidePosition;
        this.bucketLiftPosition = bucketLiftPosition;
        this.robotliftPosition = robotLiftPosition;
        this.robotlift0Position = robotLift0Position;
        this.robotlifthangPosition = robotLifthangPosition;
        this.rotationServoDirection = rotationServoDirection;
    }
}