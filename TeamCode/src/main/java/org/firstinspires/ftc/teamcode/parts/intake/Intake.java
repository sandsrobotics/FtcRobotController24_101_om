package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl> {
    public int slideTargetPosition;
    double motorPower = 0;
    private int currentSlidePos;
    //private int currentLiftPos;
    private int currentBucketPos;
    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0, 0, 0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware) {
        super(parent, "slider", () -> new IntakeControl(0, 0, 0, 0));
        setConfig(settings, hardware);
    }

    private void setSlideToHomeConfig() {
        double power = -0.125;

        getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        getHardware().horizSliderMotor.setPower(power);
        //getHardware().rightLiftMotor.setPower(power);
    }

    private void setMotorsToRunConfig() {
        getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().horizSliderMotor.setPower(IntakeHardware.slideHoldPower);
        getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //getHardware().rightLiftMotor.setPower(LifterHardware.liftHoldPower);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void slideWithPower(double power, boolean force) {
        if (Math.abs(power) < getSettings().minRegisterVal) return;

        if (power < 0)
            power *= getSettings().maxSlideSpeed;
        else
            power *= getSettings().maxSlideSpeed;

        if (force)
            setSlidePositionUnsafe(getSlidePosition() + (int) power);
        else
            setSlidePosition(getSlidePosition() + (int) power);
    }

    public void sweepWithPower(double power) {
        getHardware().intakeFlipperServo.setPower(power);
    }

    public void setSlidePosition(int position) {
        setSlidePositionUnsafe(Math.min(getSettings().maxSlidePosition, Math.max(getSettings().minSlidePosition, position)));
    }

    private void setSlidePositionUnsafe(int position) {
        slideTargetPosition = position;
        getHardware().horizSliderMotor.setTargetPosition(position);
    }

    private void changeSlidePosition(int position) {
        if (position > -1) {
            if(position > 0)
                setSlidePosition(getSettings().maxSlidePosition);
            else setSlidePosition(0);
        }
    }

    private void setBucketLiftPosition(int position) {
        if (position == -1) { // go down
            setBucketLiftPositionUnsafe(getSettings().minLiftPosition);
        } else if ( position == 1) { // go up
            setBucketLiftPositionUnsafe(getSettings().bucketMaxPos);
        }
    }

    private void setBucketLiftPositionUnsafe(int position) {
        getHardware().bucketLiftMotor.setTargetPosition(position);
    }

    public boolean isLiftInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    public int getSlidePosition() {
        return currentSlidePos;
    }

    public int getBucketLiftPosition() {
        return currentBucketPos;
    }

    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                getHardware().tiltServo.setPosition(getSettings().tiltServoDownPosition);
                break;
            case 2:
                getHardware().tiltServo.setPosition(getSettings().tiltServoUpPosition);
                break;
        }
    }

    @Override
    public void onInit() {
        setMotorsToRunConfig();
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) {
        sweepWithPower(control.sweeperPower);
        setSweepPosition(control.sweepLiftPosition);
        changeSlidePosition(control.sweepSlidePosition);
        setBucketLiftPosition(control.bucketLiftPosition);

        //slideWithPower(control.sweepSlidePosition,false);

        currentSlidePos = getHardware().horizSliderMotor.getCurrentPosition();
        currentBucketPos = getHardware().bucketLiftMotor.getCurrentPosition();
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

