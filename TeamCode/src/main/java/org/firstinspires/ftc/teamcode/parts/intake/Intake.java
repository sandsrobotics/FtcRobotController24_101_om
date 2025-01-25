package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
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

    public IntakeTasks tasks;
    protected Drive drive;

    public int slideTargetPosition;
    public int liftTargetPosition;
    private int currentSlidePos;
    private int currentBucketPos;
    public int lastSample = -1;
    public double lastSampleDistance = 10;
    private double spinnerSliderPower = 0.0;    // what is this?

    public boolean isRedGood = true;
    public boolean isYellowGood = true;
    public boolean isBlueGood = false;
    public boolean slideIsUnderControl = false;
    public boolean preventUserControl = false;
    public boolean debugMode = false;

    // this is part of the resets lift to 0 each time it hits the limit switch
    private final EdgeConsumer homingLiftZero = new EdgeConsumer();
    private final EdgeConsumer homingSlideZero = new EdgeConsumer();

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
        // apply initial position
        getHardware().spinner.setPosition(getSettings().spinnerOff);
        getHardware().flipper.setPosition(getSettings().flipperParked);
        getHardware().chute.setPosition(getSettings().chuteParked);
        getHardware().pinch.setPosition(getSettings().pinchFullOpen);
        getHardware().park.setPosition(getSettings().parkDown);
    }

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

    public boolean isSlideInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().toleranceSlide;
    }

    public boolean isLiftInTolerance() {
        return Math.abs(liftTargetPosition - currentBucketPos) <= getSettings().toleranceLift;
    }

    public int getSlidePosition() {
        return currentSlidePos;
    }

    public int getLiftPosition() {
        return currentBucketPos;
    }

    public void setSpinner(double speed) {
        getHardware().spinner.setPosition(speed);
    }

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

    public void setUserSlidePower(double power) {
        if (preventUserControl) return;
        if (power > 0 && getHardware().slideMotor.getCurrentPosition() >= getSettings().positionSlideMax) {
            setSlidePosition(getSettings().positionSlideMax, 0.25);
            return;
        }
        if (power < 0 && getHardware().slideMotor.getCurrentPosition() <= getSettings().positionSlideMin) {
            setSlidePosition(getSettings().positionSlideMin, 0.25);
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

    public boolean debugDelay() {
        if (!debugMode) return true;
        return (parent.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped) ||
                parent.buttonMgr.getState(1, ButtonMgr.Buttons.y, ButtonMgr.State.isPressed));
    }

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
        homingLiftZero.setOnRise(() -> {
            getHardware().liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setLiftPosition(10,0.125);
        });
        //homing hslide slide setup
        homingSlideZero.setOnRise(() -> {
            getHardware().slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setSlidePosition(10,0.125);
        });
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) {
        spinnerSliderPower = 0.0; // control.strafePower;
        currentSlidePos = getHardware().slideMotor.getCurrentPosition();
        currentBucketPos = getHardware().liftMotor.getCurrentPosition();

        homingLiftZero.accept(getHardware().liftZeroSwitch.getState());
        homingSlideZero.accept(getHardware().slideZeroSwitch.getState());
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

