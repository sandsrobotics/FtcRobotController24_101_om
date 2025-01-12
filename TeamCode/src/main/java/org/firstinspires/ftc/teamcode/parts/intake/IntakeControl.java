package org.firstinspires.ftc.teamcode.parts.intake;
public class IntakeControl {
    public int sweeperPower;
    public int sweepLiftPosition;
    public int sweepSlidePosition;
    public int bucketLiftPosition;
    public int specimanClawSupplier;
    public float v_SlideSupplier;
    public int intakeAngleSupplier;

    public IntakeControl(int sweeperPower, int sweeperLiftPosition, int sweepSlidePosition,
                         int bucketLiftPosition, int specimanClawSupplier, float v_SlideSupplier,
                         int intakeAngleSupplier) {
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.sweepSlidePosition = sweepSlidePosition;
        this.bucketLiftPosition = bucketLiftPosition;
        this.specimanClawSupplier = specimanClawSupplier;
        this.v_SlideSupplier = v_SlideSupplier;
        this.intakeAngleSupplier = intakeAngleSupplier;
    }
}