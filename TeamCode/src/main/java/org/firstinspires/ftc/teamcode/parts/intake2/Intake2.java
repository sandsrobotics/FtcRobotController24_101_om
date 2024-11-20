package org.firstinspires.ftc.teamcode.parts.intake2;

import org.firstinspires.ftc.teamcode.parts.intake2.hardware.IntakeHardware2;
import org.firstinspires.ftc.teamcode.parts.intake2.settings.IntakeSettings2;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;

public class Intake2 extends ControllablePart<Robot, IntakeSettings2, IntakeHardware2, IntakeControl2> {
    public int slideTargetPosition;
    double motorPower = 0;
    private double currentSlidePos = 0.5;
    private int currentLiftPos;

    //***** Constructors *****
    public Intake2(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl2(0, 0, 0, 0));
        setConfig(
                IntakeSettings2.makeDefault(),
                IntakeHardware2.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake2(Robot parent, IntakeSettings2 settings, IntakeHardware2 hardware) {
        super(parent, "slider", () -> new IntakeControl2(0, 0, 0, 0));
        setConfig(settings, hardware);
    }

    public void sweepWithPower(double power) {
        getHardware().intakeWheelServoLeft.setPower(power);
        getHardware().intakeWheelServoRight.setPower(power);
    }

    private void setBucketLiftPositionUnsafe(int position) {
        getHardware().bucketLiftMotor.setTargetPosition(position);
    }

    public boolean isLiftInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    public double getSlidePosition() {
        return currentSlidePos;
    }

    public int getRobotLiftPosition() {
        return currentLiftPos;
    }

    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                getHardware().tiltServoLeft.setPosition(getSettings().tiltServoDownPosition);
                break;
            case 2:
                getHardware().tiltServoLeft.setPosition(getSettings().tiltServoUpPosition);
                break;
        }
    }

    public void incrementAndSlide(int position) {
        switch (position) {
            case 1:
                currentSlidePos += .01;
                break;
            case 2:
                currentSlidePos -= .01;
                break;
        }
        getHardware().sliderServo.setPosition(currentSlidePos);
    }


    public void setSlidePositionServo(int position) {
        switch (position) {
            case 0:
                getHardware().sliderServo.setPosition(getSettings().minServoSlide);
                break;
            case 1:
                getHardware().sliderServo.setPosition(getSettings().maxServoSlide);
                break;
        }
    }
    @Override
    public void onInit() {

    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl2 control) {
        sweepWithPower(control.sweeperPower);
        setSweepPosition(control.sweepLiftPosition);
        setSlidePositionServo(control.sweepSlidePosition);
        //incrementAndSlide(control.sweepSlidePosition);

        currentLiftPos = getHardware().robotLift1Motor.getCurrentPosition();
    }

    @Override
    public void onStart() {
        //drive = getBeanManager().getBestMatch(Drive.class, false);
    }

    @Override
    public void onStop() {
        //drive.removeController(ContollerNames.distanceContoller);
    }
}

