package org.firstinspires.ftc.teamcode.parts.intake2;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.parts.intake2.hardware.IntakeHardware2;
import org.firstinspires.ftc.teamcode.parts.intake2.settings.IntakeSettings2;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.consumer.EdgeConsumer;

public class Intake2 extends ControllablePart<Robot, IntakeSettings2, IntakeHardware2, IntakeControl2> {
    public int slideTargetPosition;
    double motorPower = 0;
    private double currentSlidePos = 0.5;
    private double currentIntakeHeightPos = 0.5;
    private double currentRotationPos = 0.0;
    private double currentHorizontalSlidePos = 0.781;
    private int currentLiftPos;
    private final EdgeConsumer homingBucketZero = new EdgeConsumer();
    private final EdgeConsumer homingLiftZero = new EdgeConsumer();

    //***** Constructors *****
    public Intake2(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl2(0, 0, 0, 0, 0,0,0,0));
        setConfig(
                IntakeSettings2.makeDefault(),
                IntakeHardware2.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake2(Robot parent, IntakeSettings2 settings, IntakeHardware2 hardware) {
        super(parent, "slider", () -> new IntakeControl2(0, 0, 0, 0,0,0,0,0));
        setConfig(settings, hardware);
    }

    public void spinIntakeWithPower(double power) {
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

    public void setIntakeUpDown(int position) {
        switch (position) {
            case 1:
                getHardware().tiltServoLeft.setPosition(getSettings().tiltServoDownPosition);
                break;
            case 2:
                getHardware().tiltServoLeft.setPosition(getSettings().tiltServoUpPosition);
                break;
        }
    }

    public void incrementRotationServo(int direction) {
        double step = getSettings().rotationServoStep;
        double newPos = currentRotationPos + (direction * step);
        currentRotationPos = Math.max(getSettings().rotationServoMin, Math.min(getSettings().rotationServoMax, newPos));
        getHardware().rotationServo.setPosition(currentRotationPos);
    }

    public void incrementIntakeUpDown(int position) {
        switch (position) {
            case 1:
                if (currentIntakeHeightPos <= getSettings().intakeArmMax)
                    currentIntakeHeightPos += .01;
                break;
            case 2:
                if (currentIntakeHeightPos >= getSettings().intakeArmMin)
                    currentIntakeHeightPos -= .01;
                break;
        }
        getHardware().tiltServoLeft.setPosition(currentIntakeHeightPos);
    }

    public void incrementHorizontalSlide(int position) {
        switch (position) {
            case 1:
                if (currentHorizontalSlidePos <= getSettings().maxServoLeftSlide)
                    currentHorizontalSlidePos += .01;
                break;
            case 2:
                if (currentHorizontalSlidePos >= getSettings().minServoLeftSlide)
                    currentHorizontalSlidePos -= .01;
                break;
        }
        getHardware().sliderServoLeft.setPosition(currentHorizontalSlidePos);
        getHardware().sliderServoRight.setPosition(currentHorizontalSlidePos);
    }


    public void setHorizontalSlidePosition(int position) {
        switch (position) {
            case 1:
                getHardware().sliderServoLeft.setPosition(getSettings().minServoLeftSlide);
                getHardware().sliderServoRight.setPosition(getSettings().minServoRightSlide);
                break;
            case -1:
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


    public void incrementalBucketUpDown(int position) {
        if (position == 1)
            setRobotLiftPositionUnsafe(getRobotLiftPosition() - 50);
        //setBucketLiftPositionUnsafe
    }

    @Override
    public void onInit() {
        currentIntakeHeightPos = getSettings().intakeArmDefault;
        currentRotationPos = 0.0;
        setHorizontalSlidePosition(-1); // pull slide in on init
        incrementIntakeUpDown(0); // default straight up position

        //homing bucket lift setup
        homingBucketZero.setOnRise(() -> {
            getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });
        //homing robot lift setup
        homingLiftZero.setOnRise(() -> {
            getHardware().robotLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getHardware().robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl2 control) {
        spinIntakeWithPower(control.sweeperPower); // two servo intake spin fwd/reverse
        incrementIntakeUpDown(control.sweepLiftPosition); // intake angle incremental angle
        setHorizontalSlidePosition(control.sweepSlidePosition); // intake slide in/out all the way

        //Todo: Lift tower up and down, score
        //Todo: Zero lift tower on digital input
        //Todo: Zero robot lift on digital input
        //Todo: Bucket angle set
        //Todo: Slide in/out infinite positions

        // Check intake height and adjust rotation servo
        if (currentIntakeHeightPos >= 0.3) {
            currentRotationPos = 0.5;
            getHardware().rotationServo.setPosition(currentRotationPos);
        } else {
            incrementRotationServo(control.rotationServoDirection);
        }

        //Todo: test code needs control refactoring
        setRobotLiftPosition(control.robotliftPosition, control.robotlift0Position, control.robotlifthangPosition);
        homingBucketZero.accept(!getHardware().bucketLiftZeroSwitch.getState());
        homingLiftZero.accept(!getHardware().robotLiftZeroSwitch.getState());
        currentLiftPos = getHardware().robotLiftMotor.getCurrentPosition(); //0.32
        parent.opMode.telemetry.addData("Intake height", currentIntakeHeightPos);
        parent.opMode.telemetry.addData("Rotation servo position", currentRotationPos);
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

