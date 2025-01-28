package org.firstinspires.ftc.teamcode.parts.intake2;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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
    public int bucketLiftTargetPosition;
    double motorPower = 0;
    private int currentBucketLiftPos = 20;
    private double currentIntakeHeightPos = 0.5;
    private double currentRotationPos = 0.0;
    private double currentHorizontalSlidePos = 0.781;
//    private int currentLiftPos;
    // Watch for bucket lift zero
    private final EdgeConsumer homingBucketZero = new EdgeConsumer();
    protected Drive drive;
    private float strafePower = 0;
    public Intake2Tasks tasks;
    protected PositionTracker pt;
    boolean startSpecRange = false;
    public boolean isTeleop;
    public int lastSample = -1;

    //***** Constructors *****
    public Intake2(Robot parent, String modeName) {
        super(parent, "Slider", () -> new IntakeControl2(0.5, 0, 0, 0, 0, 0, 0, 0, 0));
        this.isTeleop = modeName.equalsIgnoreCase("Teleop");

        setConfig(
                IntakeSettings2.makeDefault(),
                IntakeHardware2.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public void spinIntakeWithPower(double power) {
        double finalPower = .5;

        if (power == 1) finalPower = 1;
        else if (power == -1 ) finalPower = 0.0;

        getHardware().intakeWheelServoLeft.setPosition(finalPower);
        getHardware().intakeWheelServoRight.setPosition(finalPower);
    }

    private void setBucketLiftPositionUnsafe(int position) {
        getHardware().bucketLiftMotor.setTargetPosition(position);
    }

    private void setRobotLiftPositionUnsafe(int position) {
        getHardware().robotLiftMotor.setTargetPosition(position);
    }

    public boolean isLiftInTolerance() {
        return abs(bucketLiftTargetPosition - getBucketLiftPosition()) <= getSettings().tolerance;
    }

    public int getBucketLiftPosition() {
        return currentBucketLiftPos;
    }

    public double getHSlidePosition() {
        return currentIntakeHeightPos;
    }

//    public int getRobotLiftPosition() {
//        return currentLiftPos;
//    }

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
                        getSettings().intakeArmAtSpecimen,
                        Math.min(
                                getSettings().intakeArmAtBucket,
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
                stopAllIntakeTasks();
                tasks.autoSpecimenPickupTask.restart();
                break;
            case 2: // Close position
                setLiftPosition(getSettings().specimenHangPosition,1);
                break;
            case -1: // Open position
                stopAllIntakeTasks();
                tasks.autoSpecimenHangTask.restart();
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
                //tasks.startAutoIntakeDropTask();
                break;
        }
    }

    public void stopLift() { getHardware().bucketLiftMotor.setPower(0); }
    public void setLiftPower (double m1) { getHardware().bucketLiftMotor.setPower(m1); }

    public void setRobotLiftPosition(int lift) {
        if (lift == 1) {
            setRobotLiftPositionUnsafe(7200); // top
        } else if (lift == -1) {
            setRobotLiftPositionUnsafe(1100); // bottom
        }
    }

    public void incrementalBucketUpDown(int position) {
        if (position == 1)
            setRobotLiftPositionUnsafe(getBucketLiftPosition() - 50);
        //setBucketLiftPositionUnsafe
    }

    public void strafeRobot(DriveControl control) {
        if (abs(strafePower) > .01) {
            control.power = control.power.addX(strafePower / 3);
        }
    }

    public void setLiftPosition(int position, double power) {
        if (position < 0 || position > 3000) {  // something very wrong so bail
            stopLift();
            return;
        }
        bucketLiftTargetPosition = position;
        stopLift();   // ???
        getHardware().bucketLiftMotor.setTargetPosition(bucketLiftTargetPosition);
        getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setLiftPower(power);
        //slideIsUnderControl = false;
    }

    public void stopIntakeSpin() {
        getHardware().intakeWheelServoLeft.setPosition(0.5);
        getHardware().intakeWheelServoRight.setPosition(0.5);
    }

    public int identifySampleColor() {
        float[] hsvValues = new float[3];
        NormalizedRGBA colorPlural = getHardware().colorSensor.getNormalizedColors();
        Color.colorToHSV(colorPlural.toColor(), hsvValues);
        int hue = (int) hsvValues[0];
        lastSample = 0;
        if (hue > 20 && hue < 60) lastSample = 1; // Red = 1
        if (hue > 65 && hue < 160) lastSample = 2; // Yellow = 2
        if (hue > 190) lastSample = 3; // Blue = 3
        parent.opMode.telemetry.addData("Hue", hue);
        return lastSample; // Nothing detected
    }

    public void initializeServos() {
//        parent.opMode.sleep(500);
        getHardware().tiltServo.setPosition(getSettings().intakeArmStraightUp -.05); // default straight up position
        getHardware().dropperServo.setPosition(.714);
        getHardware().specimenServo.setPosition(getSettings().specimenServoOpenPosition +.05);
        getHardware().rotationServo.setPosition(.5 +.05);
        parent.opMode.sleep(100);
        getHardware().tiltServo.setPosition(getSettings().intakeArmStraightUp);
        getHardware().specimenServo.setPosition(getSettings().specimenServoOpenPosition);
        getHardware().dropperServo.setPosition(.716);
        getHardware().rotationServo.setPosition(.5);
        parent.opMode.sleep(100);
        getHardware().dropperServo.stop();
    }

    @Override
    public void onInit() {
        currentIntakeHeightPos = getSettings().intakeArmStraightUp;
        initializeServos();
        stopIntakeSpin();
        currentRotationPos = 0.0;
        setHorizontalSlidePosition(-1); // pull slide in on init
        drive = getBeanManager().getBestMatch(Drive.class, false);
        pt = getBeanManager().getBestMatch(PositionTracker.class, false);
        tasks = new Intake2Tasks(this, parent);
        tasks.constructAllIntakeTasks();

        //homing bucket lift setup
        homingBucketZero.setOnRise(() -> {
            getHardware().bucketLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setLiftPosition(20,0.125);
            //tasks.setMotorsToRunConfig();
        });
        initializeServos();
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
        if (isTeleop) {
            spinIntakeWithPower(control.sweeperPower);
            incrementIntakeUpDown(control.sweepLiftPosition); // intake angle incremental angle
            incrementHorizontalSlide(control.sweepSlidePosition); // intake slide in/out all the way
            setBucketLiftPosition(control.bucketLiftPosition);
            setSpecimenPositions(control.specimenServoPosition);
            setRobotLiftPosition(control.robotliftPosition);

            // Check intake height and adjust rotation servo
            if (currentIntakeHeightPos >= 0.3) {
                currentRotationPos = 0.5;
                getHardware().rotationServo.setPosition(currentRotationPos);
            } else {
                incrementRotationServo(control.rotationServoDirection);
            }
        }
        strafePower = control.strafePower;
        homingBucketZero.accept(getHardware().bucketLiftZeroSwitch.getState());
        currentBucketLiftPos = getHardware().bucketLiftMotor.getCurrentPosition();
        parent.opMode.telemetry.addData("Specimen Color", getColor());
        parent.opMode.telemetry.addData("Intake height", currentIntakeHeightPos);
        parent.opMode.telemetry.addData("Rotation servo position", currentRotationPos);
        parent.opMode.telemetry.addData("bucketLiftMotor postion", getHardware().bucketLiftMotor.getCurrentPosition());
    }

    @Override
    public void onStart() {
        drive.addController(ControllerNames.distanceController, this::strafeRobot);
        drive.addController(ControllerNames.specController, this::doSpecRange);
        getHardware().dropperServo.stop();
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

    public String getColor() {
        switch (identifySampleColor()) {
            case 1:
                return "Red";
            case 2:
                return "Yellow";
            case 3:
                return "Blue";
            default:
                return "None";
        }
    }
}