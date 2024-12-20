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
    public int hangValue;

    //***** Constructors *****
    public Intake2(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl2(0, 0, 0, 0, 0,0,0));
        setConfig(
                IntakeSettings2.makeDefault(),
                IntakeHardware2.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake2(Robot parent, IntakeSettings2 settings, IntakeHardware2 hardware) {
        super(parent, "slider", () -> new IntakeControl2(0, 0, 0, 0,0,0,0));
        setConfig(settings, hardware);
    }

    public void sweepWithPower(double power) {
        getHardware().intakeWheelServoLeft.setPower(power);
        getHardware().intakeWheelServoRight.setPower(power);
    }

    private void setBucketLiftPositionUnsafe(int position) {
        getHardware().bucketLiftMotor.setTargetPosition(position);
    }

    private void setRobotLiftPositionUnsafe(int position) {
        getHardware().robotLiftMotor.setTargetPosition(position);
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
        getHardware().sliderServoLeft.setPosition(currentSlidePos);
    }


    public void setSlidePositionServo(int position) {
        switch (position) {
            case 0:
                getHardware().sliderServoLeft.setPosition(getSettings().minServoLeftSlide);
                getHardware().sliderServoRight.setPosition(getSettings().minServoRightSlide);
                break;
            case 1:
                getHardware().sliderServoLeft.setPosition(getSettings().maxServoLeftSlide);
                getHardware().sliderServoRight.setPosition(getSettings().maxServoRightSlide);
                break;
        }
    }

    public void setRobotLiftPosition(int lift, int zero, int hang) {
        if (lift == 1){
            setRobotLiftPositionUnsafe(5000); // top
        } else if (lift == -1){
            setRobotLiftPositionUnsafe(0); // bottom
        } else if(zero == 1) {
            setRobotLiftPositionUnsafe(getRobotLiftPosition() - 50);
        } else if(hang == -1){
            setRobotLiftPositionUnsafe(943);
        } else {
            setRobotLiftPositionUnsafe(getRobotLiftPosition());
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
        setRobotLiftPosition(control.robotliftPosition, control.robotlift0Position, control.robotlifthangPosition);
        hangValue = control.robotlifthangPosition;
        currentLiftPos = getHardware().robotLiftMotor.getCurrentPosition();
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

