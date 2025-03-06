package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

import static java.lang.Math.abs;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl> {

    public IntakeTasks tasks;
    protected Drive drive;
    protected Pinpoint pinpoint;
    protected PositionSolver positionSolver;

    public int slideTargetPosition;
    public int liftTargetPosition;
    private int currentSlidePos;
    private int currentLiftPos;
    public int lastSample = -1;
    public double lastSampleDistance = 10;  // in cm
    public double lastRearDistance = 323;  // in inches
    private double spinnerSliderPower = 0.0;// what is this?
    public double rangePower = 0.40; //todo: finalize
    public boolean rangeIsDone = false;
    public boolean rangeEnabled = false;
    public Vector3 adjustedDestination = null;

    // for testing PID
    public PIDFCoefficients pidf_rue = new PIDFCoefficients();
    public PIDFCoefficients pidf_rtp = new PIDFCoefficients();
//    public PIDFCoefficients sample_pidf_rue = new PIDFCoefficients();
//    public PIDFCoefficients sample_pidf_rtp = new PIDFCoefficients();
    public float pIncrement = 1;

    // for testing outtake speed
    public double testSpinnerOut = 0.0;
    public double sIncrement = 0.05;

    public boolean slideIsUnderControl = false;
    public boolean preventUserControl = false;

    // this is part of the resets lift to 0 each time it hits the limit switch
    private final EdgeConsumer homingLiftZero = new EdgeConsumer();
    private final EdgeConsumer homingSlideZero = new EdgeConsumer();

    public final Vector3 p_atObsZone = new Vector3(33.5, -56.5, 90); // p_12: Position near ObsZone for Pickup-Specimen.
    public final Vector3 p_beforeHighRung = new Vector3(2.75 - 6, -40.25 + 5 , -90); // p_19: Position before High-Rung for Hang-Specimen.

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

    public void initializeServos() {
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
        //slideTargetPosition = position;
        slideTargetPosition = !getHardware().slideZeroSwitch.getState() ? position :
                Math.max(position, getSettings().positionSlideHome);  // if the switch is pressed, the minimum is the home position
        stopSlide();   // ???
        getHardware().slideMotor.setTargetPosition(slideTargetPosition);
        setSlidePower(power);
        getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideIsUnderControl = false;
    }

    public void setLiftPosition(int position, double power) {
        if (position < getSettings().positionLiftMin || position > getSettings().positionLiftMax) {  // something very wrong so bail
            stopLift();
            return;
        }
        //liftTargetPosition = position;
        liftTargetPosition = !getHardware().liftZeroSwitch.getState() ? position :
                Math.max(position, getSettings().positionLiftHome);  // if the switch is pressed, the minimum is the home position
        stopLift();   // ???
        getHardware().liftMotor.setTargetPosition(liftTargetPosition);
        setLiftPower(power);
        getHardware().liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setHangPosition(int position, double power) {
        getHardware().robotHangMotor.setTargetPosition(position);
        getHardware().robotHangMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        getHardware().robotHangMotor.setPower(power);
    }


    public double getRangeDistance(){
        lastRearDistance = getHardware().distanceSensor.getDistance(DistanceUnit.CM);
        if (lastRearDistance > 1000) lastRearDistance = 0;
        return lastRearDistance;
    }

    // ranging - This will not maintain angle.
    // 2m distance sensor
    //
    public void doRanging(DriveControl control) {
        if(rangeEnabled) {
            double range = getRangeDistance();
            parent.opMode.telemetry.addData("range", range);
            if (range <= 7) {  //
                drive.stopRobot();
                rangeIsDone = true;
                rangeEnabled = false;
            } else { // if (range > 7)
                double currRangePower = rangePower;
//                double currRangePower = Math.min(0.6, 0.1 + ((range - 7) * 0.5));
                control.power = control.power.addY(-currRangePower); // (toward sub)
                rangeIsDone = false;
//                parent.opMode.telemetry.addData("range", range);
            }
        }
    }



    //   method to get a modified target position based on the distance/range sensor.
    public Vector3 adjustTargetPositionByRangeY(Vector3 targetPosition, double targetDistance) {
        //Returns null if a suitable position is not found, otherwise a new target position based on current position
        //Reads the distance sensor and then gets a new current pinpoint position (to do: work with positiontracker instead)
        //The X and Z will be preserved, but Y will be changed to reflect best available position data
        if (targetPosition.Z != 90 && targetPosition.Z != -90) return null;  //only currently works for pure Y
        final double acceptableDiff = 5;  //how much farther away is OK for calculations?
        final double acceptableAngle = 5;  //how far can odo angle be from target angle for calculations?
        getRangeDistance();
        parent.opMode.telemetry.addData ("ranging:", lastRearDistance);
        if (lastRearDistance <= targetDistance + acceptableDiff) {   // if it's in acceptable range for calculations...
            Vector3 currentPosition = pinpoint.getValidPosition();   // get the current odo position
            if (currentPosition == null) return null;                // if not valid odo position, return null
            if (Math.abs(currentPosition.Z - targetPosition.Z) > acceptableAngle) return null;   // return null if angle too large
            double yAdjust = (lastRearDistance - targetDistance) * -1.0 * Math.signum(targetPosition.Z);  // distance + at 90°, - at -90°
            // new position has original X and Z, but Y is based on currentPosition, targetDistance, and range
            return new Vector3(targetPosition.X, currentPosition.Y + yAdjust, targetPosition.Z);
        }
        return null;
    }

    public boolean adjustTarget(Vector3 targetPosition, double targetDistance) {
        adjustedDestination = adjustTargetPositionByRangeY(targetPosition, targetDistance);
        return adjustedDestination != null;
    }
    public void stopSlide() { getHardware().slideMotor.setPower(0); }
    public void stopLift() { getHardware().liftMotor.setPower(0); }
    public void setSlidePower (double m0) { getHardware().slideMotor.setPower(m0); }
    public void setLiftPower (double m1) { getHardware().liftMotor.setPower(m1); }

    public boolean isSlideInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().toleranceSlide;
    }

    public boolean isLiftInTolerance() {
        return Math.abs(liftTargetPosition - currentLiftPos) <= getSettings().toleranceLift;
    }

    public int getSlidePosition() {
        return currentSlidePos;
    }

    public int getLiftPosition() {
        return currentLiftPos;
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
        getHardware().park.stop();
        //stop the hang motor
        getHardware().robotHangMotor.setPower(0);
        //stop the position solver?
        positionSolver.stopSolver();
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

    public double readSampleDistance() {
        lastSampleDistance = ((DistanceSensor) getHardware().colorSensor).getDistance(DistanceUnit.CM);
        return lastSampleDistance;
    }
    public boolean sampleInIntake () {
        if (lastSampleDistance < 8) {
            return lastSampleDistance > 8;
        }
        return false;
    }
    public boolean isSamplePresent (boolean pollSensor) {
        if (!pollSensor) return lastSampleDistance <= getSettings().distSampleGood;
        return readSampleDistance() <= getSettings().distSampleGood;
    }
    public boolean isSamplePresent () {
        return isSamplePresent(true);
    }
    public boolean wasSamplePresent () {  // don't use this unless you've read the sensor very recently
        return isSamplePresent(false);
    }
//
//    public double readRearDistance() {
//        lastRearDistance = getHardware().distanceSensor.getDistance(DistanceUnit.INCH);
//        return lastRearDistance;
//    }

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
        if (sample == 1 && FlipbotSettings.isRedGood) return true;
        if (sample == 2 && FlipbotSettings.isYellowGood) return true;
        if (sample == 3 && FlipbotSettings.isBlueGood) return true;
        return false;
    }
    public boolean isSampleGood() {
        return isSampleGood(identifySampleColor());
    }

    public boolean debugDelay() {
        if (!FlipbotSettings.autonomousDebugMode) return true;
        parent.opMode.telemetry.addLine("***** Debug delay... Tap X or hold Y to continue *****");
        return (parent.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped) ||
                parent.buttonMgr.getState(1, ButtonMgr.Buttons.y, ButtonMgr.State.isPressed));
    }

    public void strafeRobot(DriveControl control) {
        if (abs(spinnerSliderPower) > .01) {
            control.power = control.power.addX(spinnerSliderPower / 3);
        }
    }

//    public void applyControlGovernor() {
//        //very prototype-y
//        // Sets maximum Driver 1 inputs proportionally based on extension of slides
//        //=1-maxGov*(MAX(eMin,MIN(E14,eMax))-eMin)/(eMax-eMin)
//        int slidePMin = getSettings().positionSlideStartIntake;
//        int slidePMax = getSettings().positionSlideMax;
//        double slideMaxReduction = 0.75;
//        double slideGov = 1 - slideMaxReduction*(clamp(currentSlidePos, slidePMin, slidePMax)-slidePMin)/(slidePMax-slidePMin);
//        int liftPMin = getSettings().positionSlideStartIntake;
//        int liftPMax = getSettings().positionLiftHangRelease;
//        double liftMaxReduction = 0.67;
//        double liftGov = 1 - liftMaxReduction*(clamp(currentLiftPos, liftPMin, liftPMax)-liftPMin)/(liftPMax-liftPMin);
//        FlipbotSettings.setControlGovernor(Math.min(slideGov,liftGov));
//    }
//    private static int clamp(int val, int min, int max) {
//        return Math.max(min, Math.min(val, max));
//    }

    @Override
    public void onInit() {
        setMotorsToRunConfig();
        if (FlipbotSettings.isAuto()) {
            initializeServos();
            if (FlipbotSettings.firstRun) {
                // the first time the servo controller comes online the positions set may be lost, so wait and try again
                FlipbotSettings.firstRun = false;
                parent.opMode.sleep(1500);
                initializeServos();
            }
        }
        pinpoint = getBeanManager().getBestMatch(Pinpoint.class, false);
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        tasks = new IntakeTasks(this, parent);
        tasks.constructAllIntakeTasks();

//        pidf_rue = getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        pidf_rtp = getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
        initSpecLiftPID();
//        initSampleLiftPID();

        // this is part of the resets lift to 0 each time it hits the limit switch
        homingLiftZero.setOnRise(() -> {
            getHardware().liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setLiftPosition(getSettings().positionLiftHome,0.5);
        });
        //homing hslide slide setup
        homingSlideZero.setOnRise(() -> {
            getHardware().slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            setSlidePosition(getSettings().positionSlideHome,0.5);
        });
    }

    public void initSpecLiftPID() {
       // if (true) return;
        //  spec run using encoder --> P=15
        pidf_rue = getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidf_rue = new PIDFCoefficients(getSettings().liftSpecP_rue,
                pidf_rue.i, pidf_rue.d, pidf_rue.f, MotorControlAlgorithm.LegacyPID);
        getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf_rue);
        // spec run to position --> P=20
        pidf_rtp = getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
        pidf_rtp = new PIDFCoefficients(getSettings().liftSpecP_rtp,
                pidf_rtp.i, pidf_rtp.d, pidf_rtp.f, MotorControlAlgorithm.LegacyPID);
        getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidf_rtp);
    }
    public void initSampleLiftPID() {
      //  if (true) return;
        // bucket run using encoder --> P=12.5
        pidf_rue = getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidf_rue = new PIDFCoefficients(getSettings().liftSampleP_rue,
                pidf_rue.i, pidf_rue.d, pidf_rue.f, MotorControlAlgorithm.LegacyPID);
        getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf_rue);
        // bucket run to position --> P=17.5
        pidf_rtp = getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
        pidf_rtp = new PIDFCoefficients(getSettings().liftSampleP_rtp,
                pidf_rtp.i, pidf_rtp.d, pidf_rtp.f, MotorControlAlgorithm.LegacyPID);
        getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidf_rtp);
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) {
        spinnerSliderPower = 0.0; // control.strafePower;
        currentSlidePos = getHardware().slideMotor.getCurrentPosition();
        currentLiftPos = getHardware().liftMotor.getCurrentPosition();
//        applyControlGovernor();

        homingLiftZero.accept(getHardware().liftZeroSwitch.getState());
        homingSlideZero.accept(getHardware().slideZeroSwitch.getState());
    }

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
//        drive.addController(Intake.ControllerNames.distanceController, this::strafeRobot);
        drive.addController(ControllerNames.distanceController, this::doRanging);
        if (FlipbotSettings.isTeleOp()) initializeServos();
        if (FlipbotSettings.isTeleOp())  tasks.startAutoHome();
    }

    @Override
    public void onStop() {
        drive.removeController(Intake.ControllerNames.distanceController);
    }

    public static final class ControllerNames {
        public static final String distanceController = "distance controller";
    }

}

