package org.firstinspires.ftc.teamcode.parts.intake;
public class IntakeControl {
    public int sweeperPower;
    public int sweepLiftPosition;
    public int robotLiftPosition;
    public int grabberPosition;
    public int launchState;
    public int ranging;
    public int centering;

    public IntakeControl(int sweeperPower, int sweeperLiftPosition,
                         int robotLiftPosition, int grabberPosition,
                         int launchState, int ranging, int centering) {
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.robotLiftPosition = robotLiftPosition;
        this.grabberPosition = grabberPosition;
        this.launchState = launchState;
        this.ranging = ranging;
        this.centering = centering;
    }
}