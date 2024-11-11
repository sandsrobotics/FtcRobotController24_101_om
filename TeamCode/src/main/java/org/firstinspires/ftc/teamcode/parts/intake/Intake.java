package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl> {
    private boolean sweepStored;
    public int slideTargetPosition;
    double motorPower = 0;
    private int currentSlidePos;
    private int currentLiftPos;

    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0, 0, 0, 0, 0, 0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware) {
        super(parent, "slider", () -> new IntakeControl(0, 0, 0, 0, 0, 0, 0));
        setConfig(settings, hardware);
    }

    private void setSlideToHomeConfig() {
        double power = -0.125;

        getHardware().sliderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        getHardware().sliderMotor.setPower(power);
        //getHardware().rightLiftMotor.setPower(power);
    }

    private void setMotorsToRunConfig() {
        getHardware().sliderMotor.setPower(IntakeHardware.slideHoldPower);
        //getHardware().rightLiftMotor.setPower(LifterHardware.liftHoldPower);
        getHardware().sliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void slideWithPower(double power, boolean force) {
        if (Math.abs(power) < getSettings().minRegisterVal) return;

        if (power < 0)
            power *= getSettings().maxDownLiftSpeed;
        else
            power *= getSettings().maxDownLiftSpeed;

        if (force)
            setSlidePositionUnsafe(getSlidePosition() + (int) power);
        else
            setSlidePosition(getSlidePosition() + (int) power);
    }

    public void sweepWithPower(double power) {
        getHardware().intakeWheelServoLeft.setPower(power);
        getHardware().intakeWheelServoRight.setPower(power);
    }

    public void setSlidePosition(int position) {
        setSlidePositionUnsafe(Math.min(getSettings().maxSlidePosition, Math.max(getSettings().minSlidePosition, position)));
    }

    private void setSlidePositionUnsafe(int position) {
        slideTargetPosition = position;
        getHardware().sliderMotor.setTargetPosition(position);
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

    public int getRobotLiftPosition() {
        return currentLiftPos;
    }

    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                sweepStored = false;
                getHardware().tiltServoLeft.setPosition(getSettings().tiltServoDownPosition);
                break;
            case 2:
                sweepStored = true;
                getHardware().tiltServoLeft.setPosition(getSettings().tiltServoUpPosition);
                break;
        }
    }

    @Override
    public void onInit() {
         //setSwingPosition(2);
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) {
        sweepWithPower(control.sweeperPower);
        setSweepPosition(control.sweepLiftPosition);

        currentSlidePos = getHardware().sliderMotor.getCurrentPosition();
        currentLiftPos = getHardware().bucketLiftMotor.getCurrentPosition();
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

