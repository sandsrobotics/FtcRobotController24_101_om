package org.firstinspires.ftc.teamcode.parts.intake2;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake2.hardware.IntakeHardware2;
import org.firstinspires.ftc.teamcode.parts.intake2.settings.IntakeSettings2;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

import static java.lang.Math.abs;

public class Intake2 extends ControllablePart<Robot, IntakeSettings2, IntakeHardware2, IntakeControl2> {
    public int slideTargetPosition;
    double motorPower = 0;
    private double currentSlidePos = 0.5;
    private double currentIntakeHeightPos = 0.5;
    private double currentRotationPos = 0.0;
    private double currentHorizontalSlidePos = 0.781;
    private int currentLiftPos;
    // Watch for bucket lift zero
    private final EdgeConsumer homingBucketZero = new EdgeConsumer();
    protected Drive drive;
    private float strafePower = 0;
    public Intake2Tasks tasks;
    protected PositionTracker pt;
    boolean startSpecRange = false;

    //***** Constructors *****
    public Intake2(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl2(0, 0, 0, 0, 0, 0, 0, 0, 0));
        setConfig(
                IntakeSettings2.makeDefault(),
                IntakeHardware2.makeDefault(parent.opMode.hardwareMap)
        );
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
        return abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    public double getSlidePosition() {
        return currentSlidePos;
    }

    public double getHSlidePosition() {
        return currentIntakeHeightPos;
    }

    public int getRobotLiftPosition() {
        return currentLiftPos;
    }

    public void incrementRotationServo(int direction) {
        double step = getSettings().rotationServoStep;
        double newPos = currentRotationPos + (direction * step);
        currentRotationPos = Math.max(getSettings().rotationServoMin, Math.min(getSettings().rotationServoMax, newPos));
        getHardware().rotationServo.setPosition(currentRotationPos);
    }

    public double getSpecRange(){
        //double range = getHardware().specDistance.getDistanceCm();
        return (4);}

    // 2m distance sensor
    public void doSpecRange(DriveControl control) {
        double inOutPower = 0.01;
        if(startSpecRange) {
            if (getSpecRange() < 3) {
                control.power = control.power.addY(inOutPower); // (away from sub)
            } else if (getSpecRange() > 5) {
                control.power = control.power.addY(-inOutPower); // (toward sub)
            }
        }
    }

    public void incrementIntakeUpDown(int direction) {

        if (getHardware().rotationServo.isDone()) {
            if (Math.abs(direction) != 0) {
                double adjustment = 0.01 * Math.signum(direction);
                currentIntakeHeightPos = Math.max(
                        getSettings().intakeArmMin,
                        Math.min(
                                getSettings().intakeArmMax,
                                currentIntakeHeightPos + adjustment
                        )
                );
                getHardware().tiltServo.setPosition(currentIntakeHeightPos);
            }
        }
    }

    public void incrementHorizontalSlide(int direction) {
        double adjustment = 0.01 * Math.signum(direction);

        currentHorizontalSlidePos = Math.max(
                getSettings().minServoLeftSlide,
                Math.min(
                        getSettings().maxServoLeftSlide,
                        currentHorizontalSlidePos + adjustment
                )
        );

        getHardware().sliderServoLeft.setPosition(currentHorizontalSlidePos);
        getHardware().sliderServoRight.setPosition(currentHorizontalSlidePos);
    }

    public void setHorizontalSlidePosition(int position) {
        switch (position) {
            case 1:
                getHardware().sliderServoLeft.setPosition(getSettings().minServoLeftSlide);
                getHardware().sliderServoLeft.setPosition(getSettings().minServoLeftSlide);
                getHardware().sliderServoRight.setPosition(getSettings().minServoRightSlide);
                break;
            case -1:
                getHardware().sliderServoLeft.setPosition(getSettings().maxServoLeftSlide);
                getHardware().sliderServoRight.setPosition(getSettings().maxServoRightSlide);
                break;
        }
    }

    public void setSpecimenPositions(int position) {
        switch (position) {
            case 1: // Open position
                tasks.startAutoSpecimenPickup();
                break;
            case 2: // Close position
                getHardware().bucketLiftMotor.setPower(1);
                getHardware().bucketLiftMotor.setTargetPosition(getSettings().specimenHangPosition);
                break;
            case -1: // Open position
                tasks.startAutoSpecimenHang();
                break;

        }
    }

    public void setBucketLiftPosition(int position) {
        switch (position) {
            case 1: // Move to maximum position Y
                tasks.startAutoBucketLift();
                break;
            case -1: // Move to minimum position A
                // check to make sure bucket is up clear of intake first
                if(getHardware().bucketLiftMotor.getCurrentPosition()>500) tasks.startAutoBucketDropper();
                break;
            case 2:
                tasks.startAutoIntakeDropTask();
                break;
        }
    }

    public void setRobotLiftPosition(int lift) {
        if (lift == 1) {
            setRobotLiftPositionUnsafe(7200); // top
        } else if (lift == -1) {
            setRobotLiftPositionUnsafe(1100); // bottom
        }
    }

    public void incrementalBucketUpDown(int position) {
        if (position == 1)
            setRobotLiftPositionUnsafe(getRobotLiftPosition() - 50);
        //setBucketLiftPositionUnsafe
    }
    public void moveRobotLiftToZero(int direction) {
        if (direction == 1) { // Right bumper pressed
            getHardware().robotLiftMotor.setTargetPosition(0);
            getHardware().robotLiftMotor.setPower(1); // Power to move
        }
    }

    public void strafeRobot(DriveControl control) {
        if (abs(strafePower) > .01) {
            control.power = control.power.addX(strafePower / 3);
        }
    }

    public void setLiftPosition(int position, double power) {
        slideTargetPosition = position;
        getHardware().bucketLiftMotor.setPower(0);
        getHardware().bucketLiftMotor.setTargetPosition(slideTargetPosition);
        getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        getHardware().bucketLiftMotor.setPower(power);
    }

    @Override
    public void onInit() {
        currentIntakeHeightPos = getSettings().intakeArmDefault;
        getHardware().tiltServo.enable();
        getHardware().tiltServo.setPosition(currentIntakeHeightPos); // default straight up position
        currentRotationPos = 0.0;
        setHorizontalSlidePosition(-1); // pull slide in on init
        drive = getBeanManager().getBestMatch(Drive.class, false);
        pt = getBeanManager().getBestMatch(PositionTracker.class, false);
        tasks = new Intake2Tasks(this, parent);
        tasks.constructAutoHome();
        tasks.constructAutoBucketLift();
        tasks.constructAutoBucketDropper();
        tasks.constructAutoSpecimenPickup();
        tasks.constructAutoSpecimenHang();
        tasks.constructIntakeDrop();
        tasks.constructRotateServoSafe();

        //homing bucket lift setup
        homingBucketZero.setOnRise(() -> {
            getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setLiftPosition(20,0.125);
            //tasks.setMotorsToRunConfig();
        });
    }

    public void stopAllIntakeTasks() {
        tasks.movementTask.runCommand(Group.Command.PAUSE);
        tasks.movementTask.getActiveRunnables().clear();    // this is the magic sauce... must be used after the PAUSE or it will stop working
    }
    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl2 control) {
        spinIntakeWithPower(control.sweeperPower); // two servo intake spin fwd/reverse
        incrementIntakeUpDown(control.sweepLiftPosition); // intake angle incremental angle
        incrementHorizontalSlide(control.sweepSlidePosition); // intake slide in/out all the way
        setBucketLiftPosition(control.bucketLiftPosition);
        moveRobotLiftToZero(control.robotLiftToZero);
        setSpecimenPositions(control.specimenServoPosition);
        //getHardware().specimenServo.setPosition(getSettings().specimenServoOpenPosition);
        setRobotLiftPosition(control.robotliftPosition);

        //Todo: raise robot lift hooks ready height above low bar
        //Todo: lower robot lift hooks just enough to lift robot

        // Check intake height and adjust rotation servo
        if (currentIntakeHeightPos >= 0.3) {
            currentRotationPos = 0.5;
            getHardware().rotationServo.setPosition(currentRotationPos);
        } else {
            incrementRotationServo(control.rotationServoDirection);
        }

        strafePower = control.strafePower;
        //Todo: test code needs control refactoring - rearrange controller A and B to suit drivers
        //homingBucketZero.accept(getHardware().bucketLiftZeroSwitch.getState());
        currentLiftPos = getHardware().robotLiftMotor.getCurrentPosition(); //0.32
        currentSlidePos = getHardware().bucketLiftMotor.getCurrentPosition();
        parent.opMode.telemetry.addData("Intake height", currentIntakeHeightPos);
        parent.opMode.telemetry.addData("Rotation servo position", currentRotationPos);
        parent.opMode.telemetry.addData("bucketLiftMotor postion", getHardware().bucketLiftMotor.getCurrentPosition());
    }

    @Override
    public void onStart() {
        drive.addController(ControllerNames.distanceController, this::strafeRobot);
        drive.addController(ControllerNames.specController, this::doSpecRange);
        tasks.startAutoHome();
    }

    @Override
    public void onStop() {
        drive.removeController(ControllerNames.distanceController);
        drive.removeController(ControllerNames.specController);
    }

    public static final class ControllerNames {
        public static final String distanceController = "distance controller";
        public static final String specController = "specimen controller";
    }
}