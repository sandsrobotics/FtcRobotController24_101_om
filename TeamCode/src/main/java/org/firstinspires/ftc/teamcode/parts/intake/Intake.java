package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

import static java.lang.Math.abs;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl> {
    public int slideTargetPosition;
    public int liftTargetPosition;
    private int currentSlidePos;
    private int currentBucketPos;
    public IntakeTasks tasks;
    public boolean isRedGood = true;
    public boolean isYellowGood = true;
    public boolean isBlueGood = false;
    public int lastSample = -1;
    public double lastSampleDistance = 10;
//    public final double goodSampleDistance = 1.5;
    public boolean slideIsUnderControl = false;
    public boolean preventUserControl = false;
    protected Drive drive;
    private double spinnerSliderPower = 0.0;
    // this is part of the resets lift to 0 each time it hits the limit switch
    private final EdgeConsumer homingVSlideZero = new EdgeConsumer();
    private final EdgeConsumer homingHSlideZero = new EdgeConsumer();
    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0, 0, 0, 0,0 ,0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware) {
        super(parent, "slider", () -> new IntakeControl(0, 0, 0, 0, 0, 0, 0));
        setConfig(settings, hardware);
    }

//    private void setSlideToHomeConfig() {
//        double power = -0.125;
//
//        getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        getHardware().slideMotor.setPower(power);
//        //getHardware().rightLiftMotor.setPower(power);
//    }

    private void setMotorsToRunConfig() {
        getHardware().slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().slideMotor.setPower(IntakeHardware.slideHoldPower);
        getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        getHardware().liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().liftMotor.setPower(IntakeHardware.liftHoldPower);
        getHardware().liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        getHardware().robotHangMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().robotHangMotor.setPower(0);
    }

    private void initializeServos() {
        // apply settings
        getHardware().spinner.setSweepTime(getSettings().spinnerSweepTime);
        getHardware().flipper.setSweepTime(getSettings().flipperSweepTime).setOffset(getSettings().flipperOffset);
        getHardware().chute.setSweepTime(getSettings().chuteSweepTime).setOffset(getSettings().chuteOffset);
        getHardware().pinch.setSweepTime(getSettings().pinchSweepTime).setOffset(getSettings().pinchOffset);
        getHardware().park.setSweepTime(getSettings().parkSweepTime).setOffset(getSettings().parkOffset);
        // apply initial position?
        getHardware().spinner.setPosition(getSettings().spinnerOff);
        getHardware().flipper.setPosition(getSettings().flipperParked);
        getHardware().chute.setPosition(getSettings().chuteParked);
        getHardware().pinch.setPosition(getSettings().pinchFullOpen);
        getHardware().park.setPosition(getSettings().parkDown);
    }

//    public void slideWithPower(double power, boolean force) {
//        if (Math.abs(power) < getSettings().minRegisterVal) return;
//
//        if (power < 0)
//            power *= getSettings().maxSlideSpeed;
//        else
//            power *= getSettings().maxSlideSpeed;
//
//        if (force)
//            setSlidePositionUnsafe(getSlidePosition() + (int) power);
//        else
//            setSlidePosition(getSlidePosition() + (int) power);
//    }

//    public void sweepWithPower(double power) {
//        getHardware().intakeFlipperServo.setPower(power);
//    }

    public void setSlidePosition(int position, double power) {
        if (position < getSettings().positionSlideOvershoot || position > getSettings().positionSlideMax) {  // something very wrong so bail
            stopSlide();
            return;
        }
        slideTargetPosition = position;
        stopSlide();   // ???
        getHardware().slideMotor.setTargetPosition(slideTargetPosition);
        getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setSlidePower(power);
        slideIsUnderControl = false;
    }

    public void setLiftPosition(int position, double power) {
        if (position < getSettings().positionLiftMin || position > getSettings().positionLiftMax) {  // something very wrong so bail
            stopLift();
            return;
        }
        liftTargetPosition = position;
        stopLift();   // ???
        getHardware().liftMotor.setTargetPosition(liftTargetPosition);
        getHardware().liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setLiftPower(power);
    }

    public void setHangPosition(int position, double power) {
        getHardware().robotHangMotor.setTargetPosition(position);
        getHardware().robotHangMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        getHardware().robotHangMotor.setPower(power);
    }

    public void stopSlide() { getHardware().slideMotor.setPower(0); }
    public void stopLift() { getHardware().liftMotor.setPower(0); }
    public void setSlidePower (double m0) { getHardware().slideMotor.setPower(m0); }
    public void setLiftPower (double m1) { getHardware().liftMotor.setPower(m1); }

//    public void setSlidePosition(int position) {
//        setSlidePositionUnsafe(Math.min(getSettings().positionSlideMax, Math.max(getSettings().positionSlideMin, position)));
//    }
//
//    private void setSlidePositionUnsafe(int position) {
//        slideTargetPosition = position;
//        getHardware().slideMotor.setTargetPosition(position);
//    }

//    private void changeSlidePosition(int position) {
//        if (position > -1) {
//            if(position > 0)
//                setSlidePosition(getSettings().maxSlidePosition);
//            else setSlidePosition(0);
//        }
//    }

//    private void setBucketLiftPosition(int position) {
//        if (position == -1) { // go down
//            setBucketLiftPositionUnsafe(getSettings().minLiftPosition);
//        } else if ( position == 1) { // go up
//            setBucketLiftPositionUnsafe(getSettings().bucketMaxPos);
//        }
//    }

//    private void setBucketLiftPositionUnsafe(int position) {
//        getHardware().liftMotor.setTargetPosition(position);
//    }

    public boolean isSlideInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().toleranceSlide;
    }

    public boolean isLiftInTolerance() {
        return Math.abs(liftTargetPosition - currentBucketPos) <= getSettings().toleranceLift;
    }

//    public void setIntakePosition(int position) {
//        /* See alternate controls concept in IntakeTeleop */
////        if (position == 2) { // safe
////            stopAllIntakeTasks();
////            tasks.dockTask.restart();
////        } else if ( position == 1) { // go up
////            stopAllIntakeTasks();
////            tasks.prepareToIntakeTask.restart();
////        }else if ( position == 3) { // go up
////            stopAllIntakeTasks();
////            tasks.autoIntakeTask.restart();
////        }else if ( position == 4) { // go up
////            stopAllIntakeTasks();
////            tasks.transferTask.restart();
////        }else if ( position == 5) { // go up
////            stopAllIntakeTasks();
////            tasks.prepareToDepositTask.restart();
////        }else if ( position == 6) { // go up
////            stopAllIntakeTasks();
////            tasks.depositTask.restart();
////        }
//    }

    public int getSlidePosition() {
        return currentSlidePos;
    }

    public int getLiftPosition() {
        return currentBucketPos;
    }

    public void setSpinner(double speed) {
        getHardware().spinner.setPosition(speed);
    }

//    public double getSpecimanClawMax() {
//        return getSettings().SpecimanClawMax;
//    }
//    public double getSpecimanClawMin() {
//        return  getSettings().specimanClawMin;
//    }
//    private void setSpecimanClaw(int position) {
//        if (position==-1) {//open specimanServo
//            getHardware().pinch.setPosition(getSpecimanClawMax());
//        } else if ( position == 1) { // close specimanServo
//            getHardware().pinch.setPosition(getSpecimanClawMin());
//        }
//    }

//    public double getIntakeAngleMin() {
//        return  getSettings().intakeAngleMin;
//
//    }
//    public double getIntakeAngleMax() {
//        return getSettings().intakeAngleMax;
//    }
//   // private void setIntakeAngle(int position) {
//  //      if (position==1) {
//  //          getHardware().flipper.setPosition(getIntakeAngleMin());
//     //   }
//   //     else if (position==-1) {
//       //     getHardware().flipper.setPosition(getIntakeAngleMax());
//     //   }
//   // }

    public void eStop() {
        preventUserControl = false;
        // stop all tasks in the intake group
        stopAllIntakeTasks();
        // stop the slides
        stopSlide();
        stopLift();
        // stop all the servos
        getHardware().spinner.stop();
        getHardware().flipper.stop();
        getHardware().chute.stop();
        getHardware().pinch.stop();
    }

    public void stopAllIntakeTasks() {
        preventUserControl = false;
        tasks.intakeTasksGroup.runCommand(Group.Command.PAUSE);
        tasks.intakeTasksGroup.getActiveRunnables().clear(); // this is the magic sauce... must be used after the PAUSE or it will stop working
    }

//    public int getV_Slide_Min() {
//        return getSettings().v_Slide_Min;
//    }

//    private void setV_Slide(int position) {
//        if (position == 2) {
//            getHardware().liftMotor.setTargetPosition(50);
//        }
//        else if (position==-2) {
//            getHardware().liftMotor.setTargetPosition(2800);
//        }
//        else if (position==-1) {
//            getHardware().liftMotor.setTargetPosition(20);
//
////            }
//        } else if ( position == 1) {
//            getHardware().liftMotor.setTargetPosition(1440);
//
////            }
//        }
//    }
    public void setUserSlidePower(double power) {
        if (preventUserControl) return;
        if (power > 0 && getHardware().slideMotor.getCurrentPosition() >= getSettings().positionSlideMax) {
            setSlidePosition(getSettings().positionSlideMax, 0.25);
           // slideIsUnderControl = false;
            return;
        }
        if (power < 0 && getHardware().slideMotor.getCurrentPosition() <= getSettings().positionSlideMin) {
            setSlidePosition(getSettings().positionSlideMin, 0.25);
           // slideIsUnderControl = false;
            return;
        }
        if (power == 0 && slideIsUnderControl) {
            setSlidePosition(getHardware().slideMotor.getCurrentPosition(), 0.25);
            return;
        }
        if (power == 0) {
            return;
        }
        slideIsUnderControl = true;
        getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setSlidePower(power);

    }

    public double sampleDistance() {
//        return ((DistanceSensor) getHardware().colorSensor).getDistance(DistanceUnit.CM);
        lastSampleDistance = ((DistanceSensor) getHardware().colorSensor).getDistance(DistanceUnit.CM);
        return lastSampleDistance;
    }
    public boolean isSamplePresent (boolean pollSensor) {
        if (!pollSensor) return lastSampleDistance <= getSettings().distSampleGood;
        return sampleDistance() <= getSettings().distSampleGood;
    }
    public boolean isSamplePresent () {
        return isSamplePresent(true);
    }
    public boolean wasSamplePresent () {  // don't use this unless you've read the sensor very recently
        return isSamplePresent(false);
    }

    public int identifySampleColor() {
        float[] hsvValues = new float[3];
        NormalizedRGBA colorPlural = getHardware().colorSensor.getNormalizedColors();
        Color.colorToHSV(colorPlural.toColor(), hsvValues);
        int hue = (int) hsvValues[0];
        lastSample = 0;
        if (hue > 20 && hue < 60) lastSample = 1; // Red = 1
        if (hue > 65 && hue < 160) lastSample = 2; // Yellow = 2
        if (hue > 160) lastSample = 3; // Blue = 3
        return lastSample; // Nothing detected
    }
    public boolean isSampleGood(int sample) {
        if (sample == 1 && isRedGood) return true;
        if (sample == 2 && isYellowGood) return true;
        if (sample == 3 && isBlueGood) return true;
        return false;
    }
    public boolean isSampleGood() {
        return isSampleGood(identifySampleColor());
    }

//    public void setParkServoUp() {
//        getHardware().park.setPosition(getSettings().parkUp);
//    }

    public void strafeRobot(DriveControl control) {
        if (abs(spinnerSliderPower) > .01) {
            control.power = control.power.addX(spinnerSliderPower / 3);
        }
    }
    @Override
    public void onInit() {
        setMotorsToRunConfig();
        initializeServos();
        tasks = new IntakeTasks(this, parent);
        tasks.constructAllIntakeTasks();

        // this is part of the resets lift to 0 each time it hits the limit switch
        homingVSlideZero.setOnRise(() -> {
            getHardware().liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setLiftPosition(10,0.125);
        });
        //homing hslide slide setup
        homingHSlideZero.setOnRise(() -> {
            getHardware().slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setSlidePosition(10,0.125);
        });
    }

    @Override
    public void onBeanLoad() {
    }


    @Override
    public void onRun(IntakeControl control) {
//        sweepWithPower(control.sweeperPower);
//        setSweepPosition(control.sweepLiftPosition);
//        changeSlidePosition(control.sweepSlidePosition);
//lk        setBucketLiftPosition(control.bucketLiftPosition);
//        setIntakePosition(control.intakePosition);
//lk        setSpecimanClaw(control.pinchPosition); // jas
//lk        setV_Slide(control.v_SlidePosition); // jas
//        setIntakeAngle(control.intakeAngleSupplier); // jas

        spinnerSliderPower = 0.0; // control.strafePower;
        currentSlidePos = getHardware().slideMotor.getCurrentPosition();
        currentBucketPos = getHardware().liftMotor.getCurrentPosition();

        homingVSlideZero.accept(getHardware().liftZeroSwitch.getState());
        homingHSlideZero.accept(getHardware().slideZeroSwitch.getState());
    }

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
        drive.addController(Intake.ControllerNames.distanceController, this::strafeRobot);
        tasks.startAutoHome();
    }

    @Override
    public void onStop() {
        drive.removeController(Intake.ControllerNames.distanceController);
    }

    public static final class ControllerNames {
        public static final String distanceController = "distance controller";
    }

}

