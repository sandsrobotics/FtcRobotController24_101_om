package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.depricated.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl> {
    public int slideTargetPosition;
    private int pixHeld;
    public boolean doTagRange = false;
    public boolean doTagCenter = false;
    private boolean armed;
    public boolean run = false;
    public boolean runRed = false;
    public boolean runBlue = false;
    public boolean runCenter = false;
    public boolean abortRange;
    private double xPos = 0;
    public double yPos = 36;
    public double lastBackDist;
    public double redSideDist;
    public double blueSideDist;
    public double sideRangeDist;
    public double ulSideDist;
    Vector3 mid;
    public boolean extraDrop;
    public boolean completeDrop;
    private boolean reverse;
    public boolean rangingHeld;
    private boolean sweepStored;
    AprilTag tag;
    public Drive drive;
    Led led;
    PositionSolver positionSolver;
    PositionTracker positionTracker;
    boolean isTop;
    double motorPower = 0;
    private int pixLine = 0;
    public int dropCounter;
    private double backDist;
    private final int[] pixLineToPos = {1000, 1200, 1400, 1800, 2200, 2600, 3000, 3000, 3000};
    private final Group movementTask = new Group("auto movement", getTaskManager());
    private final TimedTask autoDropTask = new TimedTask(TaskNames.autoDrop, movementTask);
    private final TimedTask autoGrabTask = new TimedTask(TaskNames.autoGrab, getTaskManager());
    private final TimedTask autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
    private final TimedTask autoDockTask = new TimedTask(TaskNames.autoDock, getTaskManager());
    private final TimedTask autoArmTask = new TimedTask(TaskNames.autoArm, getTaskManager());
    private final TimedTask autoStoreTask = new TimedTask(TaskNames.autoStore, getTaskManager());
    private final TimedTask finishDropTask = new TimedTask(TaskNames.autoFinishDrop, getTaskManager());
    private final TimedTask foundRangeTask = new TimedTask(TaskNames.autoFoundRange, getTaskManager());
    private final TaskEx runCenterTask = new TaskEx(TaskNames.autoRunCenter, getTaskManager());


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

    private void preAutoMove() {
        triggerEvent(ControllablePart.Events.stopControllers);
    }

    private void postAutoMove() {
        triggerEvent(ControllablePart.Events.startControllers);
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
            power *= getSettings().maxDownSlideSpeed;
        else
            power *= getSettings().maxUpSlideSpeed;

        if (force)
            setSlidePositionUnsafe(getSlidePosition() + (int) power);
        else
            setSlidePosition(getSlidePosition() + (int) power);
    }

    public void sweepWithPower(double power) {
        getHardware().sweeperMotor.setPower(.55 * power);
        if (power != 0) {
            topDist = getHardware().topSensor.getDistance(DistanceUnit.CM);
            botDist = getHardware().botSensor.getDistance(DistanceUnit.CM);
            setLeds2(hasPixels());
        }
    }

    public void setSlidePosition(int position) {
        setSlidePositionUnsafe(Math.min(getSettings().maxSlidePosition, Math.max(getSettings().minSlidePosition, position)));
    }

    private void setSlidePositionUnsafe(int position) {
        slideTargetPosition = position;
        getHardware().sliderMotor.setTargetPosition(position);
    }

    private void setSwingPosition(int position) {
        switch (position) {
            case 1:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftDropPosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftDropPosition);
                break;
            case 2:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftSafePosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftSafePosition);
                break;
            case 3:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftMaxPosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftMaxPosition);
                break;
            case 4:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftActualMaxPosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftActualMaxPosition);
                break;
        }
    }

    private void setRobotLiftPositionUnsafe(int position) {
        getHardware().robotLiftMotor.setTargetPosition(position);
    }

    public boolean isLiftInTolerance() {
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    private int currentSlidePos;
    private int currentLiftPos;
    private double topDist;
    private double botDist;

    public int getSlidePosition() {
        return currentSlidePos;
    }

    public int getRobotLiftPosition() {
        return currentLiftPos;
    }

    public double getTopPixelDist() {
        return topDist;
    }

    public double getBottomPixelDist() {
        return botDist;
    }

    public double getBackDist() {
        return backDist == 0.0 ? 7.0 : Math.min(backDist, 30.0);
    }

    public double getBlueSideDist(){return blueSideDist;}
    public double getRedSideDist(){return redSideDist;}

    public boolean pixHeld(){
        if (getTopPixelDist() < 2.5 && getBottomPixelDist() < 2.5)
            pixHeld++;
        else
            pixHeld = Math.max(pixHeld - 1, 0);

        return pixHeld == 4;
    }

    public int hasPixels() {
        if (getTopPixelDist() < 2.5 && getBottomPixelDist() < 2.5)
            return 2;
        else if (getTopPixelDist() > 2.5 && getBottomPixelDist() < 2.5)
            return 1;
        else
            return 0;
    }

    public boolean checkPixels(int pix){
        return hasPixels() == pix;
    }

    public int getLedBucket(RevColorSensorV3 sensor) {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        //parent.opMode.telemetry.addData("Hue", hsvValues[0]);
        int hue = (int) hsvValues[0];
        if (hue < 120) return 1; //yellow
        if (hue >= 120 && hue <= 150) return 2; //green
        if (hue > 180) return 3; //purple
        else return 4; //white
    }

    public void setLeds2(int pixels) {
        if (pixels == 1) {
            led.setBottomGroup2(getLedBucket(getHardware().botSensor));
            led.setTopGroup2(0);
        } else if (pixels == 2) {
            led.setBottomGroup2(getLedBucket(getHardware().botSensor));
            led.setTopGroup2(getLedBucket(getHardware().topSensor));
        } else {
            led.setBottomGroup2(0);
            led.setTopGroup2(0);
        }
    }

    public void setLaunchAngle(int position) {
        switch (position) {
            case 1:
                getHardware().launchServoAngle.setPosition(getSettings().launchAngleArmPosition);
                break;
            case 2:
                getHardware().launchServoAngle.setPosition(getSettings().launchAngleStorePosition);
                break;
        }
    }

    public void setLaunchState(int state) {
        if (state == 1) {
            getHardware().launchServoRelease.setPosition(getSettings().launchReleaseUnlockPosition);
            //            case 0:
//                getHardware().launchServoRelease.setPosition(getSettings().launchReleaseLockPosition);
//                break;
        }
    }

    public void setGrabPosition(int position) {
        switch (position) {
            case 1:
                getHardware().grabberServo.setPosition(getSettings().grabberOpenPosition);
                break;
            case 2:
                getHardware().grabberServo.setPosition(getSettings().grabberGripOnePosition);
                break;
            case 3:
                getHardware().grabberServo.setPosition(getSettings().grabberGripTwoPosition);
                break;
            case 4:
                getHardware().grabberServo.setPosition(getSettings().grabberPrimePosition);
                break;
        }
    }

    public int getPix() {
        return pixLine;
    }

    public void setPix(int pix) {
        if (pix > getSettings().maxPixLine) pix = getSettings().maxPixLine;
        else if (pix <= 0) pix = 0;
        this.pixLine = pix;
    }

    public void robotLiftWithPower(double power) {
        motorPower = power;

        if (power > 0) { // going up
//            isTop = false;
//            if (getHardware().liftLowLimitSwitch.getState())
//                motorPower = 0.0;
        } else if (power < 0) { // going down
//            if (getHardware().robotLiftMotor.getCurrent(CurrentUnit.MILLIAMPS) > 5500 || isTop) {
//                isTop = true;
//                motorPower = 0.0;
//            }
            if(!sweepStored)
                setSweepPosition(4);
        }
        getHardware().robotLiftMotor.setPower(motorPower);
    }

    public void constructAutoHome() {
        autoHomeTask.autoStart = false;

        autoHomeTask.addStep(() -> setGrabPosition(1));
        autoHomeTask.addStep(() -> setSwingPosition(2));

        autoHomeTask.addStep(this::setSlideToHomeConfig);

        autoHomeTask.addTimedStep(() -> {
            parent.opMode.telemetry.addData("homing", getHardware().slideLowLimitSwitch.getState());
        }, () -> !getHardware().slideLowLimitSwitch.getState(), 10000);
        autoHomeTask.addStep(() -> {
            getHardware().sliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            getHardware().sliderMotor.setTargetPosition(0);
            //getHardware().rightLiftMotor.setTargetPosition(0);

            slideTargetPosition = 0;

            setMotorsToRunConfig();
        });
    }

    public void startAutoHome() {
        autoHomeTask.restart();
    }

    public void addAutoHomeToTask(TimedTask task) {
    }

    public void constructAutoArm() {
        autoArmTask.autoStart = false;

        autoArmTask.addStep(this::preAutoMove);
        autoArmTask.addTimedStep(() -> robotLiftWithPower(1), 1500);
//        autoArmTask.addStep(() -> setLaunchAngle(1));
        autoArmTask.addStep(this::postAutoMove);
        autoArmTask.addStep(() -> triggerEvent(Events.armComplete));
    }
//
    public void startAutoArm() {
        autoArmTask.restart();
    }
//
    public void constructAutoStore() {
        autoStoreTask.autoStart = true;

        autoStoreTask.addTimedStep(()->robotLiftWithPower(-.2), 750);
//        autoStoreTask.addStep(() -> triggerEvent(Events.storeComplete));
    }

    public void startAutoStore() {
        armed = false;
        autoStoreTask.restart();
    }

    public void constructAutoGrab() {
        autoGrabTask.autoStart = false;

//        autoGrabTask.addStep(this::preAutoMove);
        autoGrabTask.addStep(()->setGrabPosition(reverse ? 3 : 1));
        autoGrabTask.addTimedStep(() -> sweepWithPower(reverse ? 1 : -1), () -> !reverse && hasPixels() == 2, reverse ? 100 : 4000); // tjk
        autoGrabTask.addTimedStep(() -> sweepWithPower(reverse ? 1 : 0), reverse ? 500 : 0); // tjk
//        autoGrabTask.addStep(this::postAutoMove);
        autoGrabTask.addStep(() -> triggerEvent(Events.grabComplete));
    }

//    public void constructAutoGrab() {
//        autoGrabTask.addStep(()-> sweepWithPower(.5), ()->hasPixels() == 2);
//        autoGrabTask.addDelay(1000);
//        autoGrabTask.addStep(()-> sweepWithPower(0));
//    }

    public void startAutoGrab() {
        autoGrabTask.restart();
    }

    public void addAutoGrabToTask(TaskEx task) {
        task.addStep(autoGrabTask::restart);
//        task.waitForEvent(eventManager.getContainer(Events.grabComplete));
    }

    public void addAutoGrabToTask(TaskEx task, boolean reverse) {
        task.addStep((Runnable) () -> this.reverse = reverse);
        addAutoGrabToTask(task);
    }

    public void addAutoGrabToTask(TaskEx task, boolean reverse, int grabTime) {
        task.addStep((Runnable) () -> this.reverse = reverse);
        addAutoGrabToTask(task);
    }

    public void constructAutoDock() {
        autoDockTask.autoStart = false;

        autoDockTask.addStep(this::preAutoMove);
        autoDockTask.addStep(() -> setGrabPosition(1));
        autoDockTask.addDelay(150);
        autoDockTask.addStep(() -> drive.addController("Move to closer pixel drop position", (control) -> control.power = control.power.addY(.6)));
        autoDockTask.addDelay(75);
        autoDockTask.addStep(() -> drive.removeController("Move to closer pixel drop position"));
        autoDockTask.addStep(() -> setSwingPosition(getSlidePosition() > 740 ? 2 : 1));
        autoDockTask.addDelay(350);
        autoDockTask.addStep(() -> setSlidePosition(0));
        autoDockTask.addStep(this::postAutoMove);
        autoDockTask.addStep(() -> triggerEvent(Lifter.Events.dockComplete));
    }

    public void startAutoDock() {
        autoDockTask.restart();
    }

    public void addAutoDockToTask(TaskEx task) {
        task.addStep(autoDockTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dockComplete));
    }

    public void constructAutoDrop() {
        autoDropTask.autoStart = false;

//        autoDropTask.addStep(this::preAutoMove);
//        autoDropTask.addStep(()-> setGrabPosition(3));
        autoDropTask.addStep(() -> setSlidePosition(pixLineToPos[pixLine]));
        autoDropTask.addDelay(700);
        autoDropTask.addStep(() -> setSwingPosition(getPix() == 8 ? 4 : getPix() == 7 ? 3 : 1));
        autoDropTask.addStep(() -> setGrabPosition(4));
        autoDropTask.addStep((Runnable) ()->completeDrop = true);
//        autoDropTask.addStep(this::postAutoMove);
        autoDropTask.addStep(() -> triggerEvent(Lifter.Events.dropComplete));
    }

    public void startAutoDrop() {
        autoDropTask.restart();
    }

    public void addAutoDropToTask(TaskEx task) {
        task.addStep(autoDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dropComplete));
    }
    public void constructFinishDrop() {
        finishDropTask.autoStart = false;

        finishDropTask.addStep(this::preAutoMove);
        finishDropTask.addStep(() -> setGrabPosition((!extraDrop || dropCounter == 1) ? 1 : 2)); //change to 2 when grabbing extra pix in auto
        finishDropTask.addDelay(200); // lower when plunger fixed
        finishDropTask.addStep(() -> drive.addController("Move to closer pixel drop position", (control) -> control.power = control.power.addY(.6)));
        finishDropTask.addDelay(65);
        finishDropTask.addStep(() -> drive.removeController("Move to closer pixel drop position"));
        finishDropTask.addStep(this::postAutoMove);
        finishDropTask.addStep(() -> triggerEvent(Events.finishDropComplete));
    }

    public void startFinishDrop() {
        finishDropTask.restart();
    }

    public void addFinishDropToTask(TaskEx task, boolean extraDrop) {
        task.addStep((Runnable) ()-> this.extraDrop = extraDrop);
        task.addStep(finishDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.finishDropComplete));
    }

    public void constructFoundRange() {
        foundRangeTask.autoStart = false;

        foundRangeTask.addStep(() -> drive.addController("Move to closer distance position", (control) -> control.power = control.power.addY(.6)));
        foundRangeTask.addDelay(35);
        foundRangeTask.addStep(() -> drive.removeController("Move to closer distance position"));

        foundRangeTask.addStep(() -> triggerEvent(Events.foundRangeComplete));
    }

    public void startFoundRange() {
        foundRangeTask.restart();
    }

    public void addFoundRangeToTask(TaskEx task, boolean side, double distance) {
//        task.addStep(()->sideRangeDist = distance);
//        task.addStep((Runnable) () -> runRed = side);
        task.addStep(foundRangeTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.foundRangeComplete));
    }

    public void addFoundRangeToTask(TaskEx task) {
        task.addStep(foundRangeTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.foundRangeComplete));
    }

    public void startRunCenter(){
        runCenterTask.restart();
    }

    public void constructRunCenter() {
        runCenterTask.autoStart = false;

        runCenterTask.addStep(this::preAutoMove);
        runCenterTask.addStep(()->{
            mid = new Vector3(xPos, yPos, 180);
            positionSolver.setNewTarget(mid, true);
        });
//        positionSolver.addMoveToTaskEx(mid, runCenterTask);
        runCenterTask.addStep(this::postAutoMove);
        runCenterTask.addStep(()->triggerEvent(Events.runCenterCompletee));
    }

    public void addRunCenterToTask(TaskEx task){
        task.addStep(runCenterTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.runCenterCompletee));
    }

    public void setRunCenter(boolean run) {
        runCenter = run;
    }

    public void setRanging(int doRanging) {
        switch (doRanging) {
            case 0:
                doTagRange = false;
                break;
            case 1:
                doTagRange = true;
                break;
        }
    }

    public void setCentering(int doCentering) {
        switch (doCentering) {
            case 0:
                doTagCenter = false;
                break;
            case 1:
                doTagCenter = true;
                break;
        }
    }


    public void doTagRanging(DriveControl control) {
        final double desiredAutoDistance = 6.8;
        final double desiredTeleDistance = 6.35;
        final double xPower = 0.08;
        final double yPower = 0.055;
        final double zPower = 0.01;
        final double yAutoPower = 0.053;
        boolean atDist = Math.abs(Math.min(55, getRedSideDist()) - sideRangeDist) <= 1.0;
        boolean atBDist = Math.abs(Math.min(55, getBlueSideDist()) - sideRangeDist) <= 1.0;
//        final double sideRangeDist = 15.0;

        if(runRed){
            if(!atDist && !(getRedSideDist() < sideRangeDist - 18)){
                control.power = control.power.addX((Math.min(60, getRedSideDist()) - sideRangeDist) * -xPower);
            } else {
                runRed = false;
            }
        }
        else if(runBlue){
            if(!atBDist && !(getBlueSideDist() > sideRangeDist - 18)){
                control.power = control.power.addX((Math.min(60, getBlueSideDist()) - sideRangeDist) * xPower);
            } else {
                runBlue = false;
            }
        }

        if (doTagCenter) {
            if (tag.desiredTag != null) {
                if(tag.desiredTag.id <= 3) yPos = 36;
                else yPos = -36;
                tag.updatePositionWithTag();
                xPos = positionTracker.getCurrentPosition().X;
            }
        } else if (doTagRange && !abortRange) {
            if(getBackDist() - lastBackDist >= 7)
                abortRange = true;
            if(rangingHeld) {
                if (getHardware().grabberLimitSwitch.getState()) {
                    control.power = control.power.addY((Math.min(getBackDist(), 15) - desiredTeleDistance) * -yPower);
                    led.setBottomGroup2(0);
                    led.setTopGroup2(0);
                } else if (!getHardware().grabberLimitSwitch.getState()) {
                    extraDrop = true;
                    startFinishDrop();
                    led.setBottomGroup2(2);
                    led.setTopGroup2(2);
                    rangingHeld = false;
                }
            }
        }
        else if (run) {
            if (getHardware().grabberLimitSwitch.getState()) {
                control.power = control.power.addY((Math.min(getBackDist(), 15) - desiredAutoDistance) * -yAutoPower);
            } else if (!getHardware().grabberLimitSwitch.getState()) {
                run = false;
            }
        }
    }


    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                sweepStored = false;
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoDownPosition);
                break;
            case 2:
                sweepStored = false;
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoStackPosition);
                break;
            case 3:
                sweepStored = false;
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoStackTopPosition);
                break;
            case 4:
                sweepStored = true;
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoStorePosition);
                break;
        }
    }


    @Override
    public void onInit() {
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, true);

        // task constructors
        constructAutoDrop();
        constructAutoDock();
        constructAutoGrab();
        constructAutoHome();
        constructAutoArm();
        constructAutoStore();
        constructFinishDrop();
        constructFoundRange();
        constructRunCenter();

        setSwingPosition(2);
//        setLaunchAngle(2);
        setLaunchState(0);
        lastBackDist = 322;
        rangingHeld = true;
        backDist = getHardware().backSensor.getDistance(DistanceUnit.INCH);
        redSideDist = getHardware().redSensor.getDistance(DistanceUnit.INCH);
        blueSideDist = getHardware().blueSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) { //TODO separate keeping Slider motor position from onRun
        if(doTagRange || run)
            backDist = getHardware().backSensor.getDistance(DistanceUnit.INCH);
        if(runRed)
            redSideDist = getHardware().redSensor.getDistance(DistanceUnit.INCH);
        if(runBlue)
            blueSideDist = getHardware().blueSensor.getDistance(DistanceUnit.INCH);

        sweepWithPower(control.sweeperPower);
        setGrabPosition(control.grabberPosition);
        robotLiftWithPower(control.robotLiftPosition);
        setLaunchState(control.launchState);
        setSweepPosition(control.sweepLiftPosition);
        setRanging(control.ranging);
        setCentering(control.centering);

        currentSlidePos = getHardware().sliderMotor.getCurrentPosition();
        currentLiftPos = getHardware().robotLiftMotor.getCurrentPosition();
//        parent.opMode.telemetry.addData("Motor amps", getHardware().robotLiftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        parent.opMode.telemetry.addData("back dist (red)", getBackDist());
//        parent.opMode.telemetry.addData("side dist (blue)", getBlueSideDist());
        parent.opMode.telemetry.addData("abort range", abortRange);
        parent.opMode.telemetry.addData("limit switch pressed: ", !getHardware().grabberLimitSwitch.getState());

        if(lastBackDist != 322)
            lastBackDist = getBackDist();
    }

    @Override
    public void onSettingsUpdate(IntakeSettings IntakeSettings) {}

    @Override
    public void onHardwareUpdate(IntakeHardware IntakeHardware) {}

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
        tag = getBeanManager().getBestMatch(AprilTag.class, false, true);
        led = getBeanManager().getBestMatch(Led.class, false);

        drive.addController(Intake.ContollerNames.distanceContoller, this::doTagRanging);
        dropCounter = 0;
//        startAutoStore();
    }

    @Override
    public void onStop() {
        drive.removeController(ContollerNames.distanceContoller);
    }

    public static final class ContollerNames {
        public static final String distanceContoller = "distance controller"; //TODO make better
    }

    public static final class TaskNames {
        public final static String autoGrab = "auto grab";
        public final static String autoDrop = "auto drop";
        public final static String autoHome = "auto home";
        public final static String autoDock = "auto dock";
        public final static String autoArm = "auto arm";
        public final static String autoStore = "auto store";
        public final static String autoFinishDrop = "auto finish drop";
        public final static String autoFoundRange = "auto found range";
        public final static String autoRunCenter = "auto run center";
    }

    public static final class Events {
        public static final String dockComplete = "DOCK_COMPLETE";
        public static final String grabComplete = "GRAB_COMPLETE";
        public static final String dropComplete = "DROP_COMPlETE";
        public static  final String homeComplete = "HOME_COMPLETE";
        public static final String armComplete = "ARM_COMPLETE";
        public static final String storeComplete = "STORE_COMPLETE";
        public static final String finishDropComplete = "DROP_FINISH_COMPLETE";
        public static final String foundRangeComplete = "FOUND_RANGE_COMPLETE";
        public static final String runCenterCompletee = "RUN_CENTER_COMPLETE";
    }
}

