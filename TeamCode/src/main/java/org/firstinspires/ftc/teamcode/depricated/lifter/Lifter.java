package org.firstinspires.ftc.teamcode.depricated.lifter;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.depricated.lifter.hardware.LifterHardware;
import org.firstinspires.ftc.teamcode.depricated.lifter.settings.LifterSettings;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.utils.PID;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

public class Lifter extends ControllablePart<Robot, LifterSettings, LifterHardware, LifterControl>{
    private final int[] poleToPos = {0,531,1346,2145}; //TODO move to settings

    private boolean grabberClosed = false;

    private final Group movementTask = new Group("auto movement",getTaskManager());
    private final TimedTask autoDockTask = new TimedTask(TaskNames.autoDock, getTaskManager());

    private final TimedTask autoPreDropTask = new TimedTask(TaskNames.preAutoDrop, movementTask);
    private final TimedTask autoDropTask = new TimedTask(TaskNames.autoDrop, movementTask);
    private final TimedTask autoPreDrop2Task = new TimedTask(TaskNames.autoDrop + "2", movementTask);

    private final TimedTask autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);

    private final TimedTask autoMoveToCone = new TimedTask(TaskNames.autoMoveToCone, getTaskManager());
    PID autoMoveToConePID = new PID();

    private final int[] coneToPos = {0,110,220,350,450}; //TODO move to settings


    private Drive drive;
    boolean startConeRange = true;
    private final TimedTask autoGrabTask = new TimedTask(TaskNames.autoGrab, getTaskManager());
    public int dockDelay = 600; //abi:110, zach:600
    private int cone; //the current cone of the stack(useful for autonomous)
    private int pole; //the pole height(0 - terminal, 1 - low, 2 - mid, 3 - high)
    private int liftTargetPosition;
    private PositionSolver positionSolver;
    private PositionTracker positionTracker;
    private int timesInPoleRange = 0;
    private boolean predrop = false;

    private final EdgeConsumer homingEdge = new EdgeConsumer();

    //***** Constructors *****
    public Lifter(Robot parent) {
        super(parent, "lifter", () -> new LifterControl(0,0,true));
        setConfig(
                LifterSettings.makeDefault(),
                LifterHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Lifter(Robot parent, LifterSettings settings, LifterHardware hardware){
        super(parent, "lifter", () -> new LifterControl(0,0,true));
        setConfig(settings, hardware);
    }

    private void preAutoMove(){
        triggerEvent(ControllablePart.Events.stopControllers);
        setGrabberClosed();
    }
    private void postAutoMove(){
        triggerEvent(ControllablePart.Events.startControllers);
    }
    private double coneDist;

    public double getDistToCone(){
        return coneDist - (cone == 0 ? 3.5 : cone == 1 ? 2.6 : 1.17);
    }

    public boolean isConeInRange(){
        return getDistToCone() < 2;
    }

    public boolean isConeInRangeTeleOp(){
        return getDistToCone() < 3 && !isGrabberClosed();
    }
    boolean autoDropRun = false;

    public void startAutoMoveToCone(){
        autoMoveToCone.restart();
    }

    public void addAutoMoveToConeToTaskEx(TaskEx taskEx){
        taskEx.addStep(autoMoveToCone::restart);
        taskEx.addStep(autoMoveToCone::isDone);
    }

    private void constructAutoMoveToCone(){
        autoMoveToCone.autoStart = false;

        autoMoveToCone.addStep(() -> {
            drive.addController("Move to cone", (control) -> control.power = control.power.addY(.07 * getDistToCone()));
        });
        autoMoveToCone.addTimedStep(() -> {parent.opMode.telemetry.addData("moving to cone", true);}, this::isConeInRange, 1500);
        autoMoveToCone.addStep(() -> drive.removeController("Move to cone"));
    }

    public void startAutoPreDrop(){
        autoPreDropTask.restart();
        predrop = true;
    }

    public void addAutoPreDropToTask(TaskEx task){
        task.addStep(autoPreDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.preDropComplete));
    }

    public void forceAddAutoPreDropToTask(TaskEx task){
        task.addStep(autoPreDropTask::restart);
    }

    public void addAutoPreDropToTask(TaskEx task, int pole, boolean wait){
        task.addStep(() -> this.pole = pole);

        task.addStep(autoPreDropTask::restart);
        if(wait)
            task.waitForEvent(eventManager.getContainer(Events.preDropComplete));
    }

    private final TimedTask coneRangeingTask = new TimedTask(TaskNames.coneMeasureRanges, getTaskManager());

    private int ultraRangeModule = 0; // keeps track of measuring ranges

    private double leftDist;
    private double rightDist;
    private double midDist;


    public void liftWithPower(double power, boolean force){
        if(Math.abs(power) < getSettings().minRegisterVal) return;

        if(power < 0)
            power *= getSettings().maxDownLiftSpeed;
        else
            power *= getSettings().maxUpLiftSpeed;

        if(force)
            setLiftPositionUnsafe(getHardware().leftLiftMotor.getCurrentPosition() + (int)power);
        else
            setLiftPosition(getHardware().leftLiftMotor.getCurrentPosition() + (int)power);
    }

    public void setLiftPosition(int position){
        setLiftPositionUnsafe(Math.min(getSettings().maxLiftPosition, Math.max(getSettings().minLiftPosition, position)));
    }

    private void setLiftPositionUnsafe(int position){
        //int diff = getHardware().rightLiftMotor.getCurrentPosition() - getHardware().leftLiftMotor.getCurrentPosition();

        liftTargetPosition = position;

        getHardware().leftLiftMotor.setTargetPosition(position);
        //getHardware().rightLiftMotor.setTargetPosition(position + diff);
    }

    public boolean isLiftInTolerance(){
        return Math.abs(liftTargetPosition - getLiftPosition()) <= getSettings().tolerance;
    }

    public void setLiftToTop(){
        setLiftPosition(getSettings().maxLiftPosition);
    }

    public void setLiftToBottom(){
        setLiftPosition(getSettings().minLiftPosition);
    }

    public int getLiftPosition(){
        return getHardware().leftLiftMotor.getCurrentPosition();
    }

    public double getRightUltra(){return rightDist;}
    public double getLeftUltra(){return leftDist;}
    public double getMidUltra(){return midDist;}

    public void turnWithPower(double power){
        setTurnPosition(getCurrentTurnPosition() + power);
    }

    public void setTurnPosition(double position){
        position = Math.min(getSettings().turnServoMaxPosition, Math.max(getSettings().turnServoMinPosition, position));

        getHardware().leftTurnServo.setPosition(position);
        getHardware().rightTurnServo.setPosition(position + getSettings().rightTurnServoOffset);
    }

    public double getCurrentTurnPosition(){
        return getHardware().leftTurnServo.getPosition();
    }
/*
    public void setGrabberPower(double power){
        getHardware().leftGrabServo.setPower(power);
        getHardware().rightGrabServo.setPower(power);
    }
*/
    public void setGrabberClosed(){
        getHardware().grabServo.setPosition(getSettings().grabberServoClosePos);
        grabberClosed = true;
    }

    public void setGrabberOpen(boolean wideOpen){
        grabberClosed = false;
        getHardware().grabServo.setPosition(wideOpen ? getSettings().grabberServoWideOpenPos : getSettings().grabberServoOpenPos);
    }

    public boolean isGrabberClosed() {
        return grabberClosed;
    }

    public void isStackSafe(){

    }

    public int getCone() {
        return cone;
    }

    public void setCone(int cone) {
        if(cone > getSettings().maxCones) cone = getSettings().maxCones;
        else if(cone < 0) cone = 0;
        this.cone = cone;
    }

    public int getPole() {
        return pole;
    }

    public void setPole(int pole) {
        if(pole > getSettings().maxPoles) pole = getSettings().maxPoles;
        else if(pole < 0) pole = 0;
        this.pole = pole;
    }

    private void constructAutoPreDrop(){
        autoPreDropTask.autoStart = false;


        autoPreDropTask.addStep(() -> {
            LifterControl.flipOpen = 0;
            setGrabberClosed();
        });
        autoPreDropTask.addStep(this::preAutoMove);
       // height should be 2060 for high, (-200 for clearance)
        //changed turn pos from .286 to .141 because couldn't open all the way without hititng

//        autoPreDropTask.addDelay(100);
        autoPreDropTask.addStep(()->setLiftPosition(poleToPos[pole]));
        autoPreDropTask.addDelay(100);
        autoPreDropTask.addStep(()->{
            if(pole == 0)
                setTurnPosition(0.34);
            else
                setTurnPosition(.27);
        });
        autoPreDropTask.addStep(this::isLiftInTolerance);
        autoPreDropTask.addStep(this::postAutoMove);
        autoPreDropTask.addStep(() -> triggerEvent(Events.preDropComplete));
    }

    public void startAutoDrop(){
        autoDropTask.restart();
        predrop = false;
    }

    private void constructAutoDrop2(){
        autoPreDrop2Task.autoStart = false;

        autoPreDrop2Task.addStep(this::preAutoMove);
        autoPreDrop2Task.addStep(()->setLiftPosition(poleToPos[pole] + 50));
        autoPreDrop2Task.addDelay(100);
        autoPreDrop2Task.addStep(()->setTurnPosition(.17));
        //autoPreDrop2Task.addStep(()->setLiftPosition(poleToPos[pole]));
        //autoPreDrop2Task.addStep(this::isLiftInTolerance);
        //autoPreDrop2Task.addDelay(500);
        //autoPreDrop2Task.addStep(()->setGrabberOpen(false));
        autoPreDrop2Task.addStep(this::postAutoMove);
        autoPreDrop2Task.addStep(() -> triggerEvent(Events.dropComplete));
    }

    public void startAutoDrop2(){
        autoPreDrop2Task.restart();
        predrop = false;
    }

    public void addAutoDropToTask(TaskEx task){
        task.addStep(autoDropTask::restart);
//        task.waitForEvent(Events.dropComplete, eventManager, () -> {});
    }

    /**
     * is the lifter turn in a position where it can move to the front without closing grabber
     */
    public boolean isLiftTurnSafe(){
        return getCurrentTurnPosition() > 0.7;
    }

    public void startAutoDock(){
        autoDockTask.restart();
        predrop = false;
    }

    public void addAutoDockToTask(TaskEx task){
        task.addStep(autoDockTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dockComplete));
    }

    public void addAutoDockToTask(TaskEx task, int cone){
        task.addStep(() -> this.cone = cone);
        addAutoDockToTask(task);
    }

    /**
     * MUST RUN autoDropPre before
     */
    private void constructAutoDrop(){
        autoDropTask.autoStart = false;

        autoDropTask.addStep(this::preAutoMove);
        autoDropTask.addStep(()->setTurnPosition(.17));
        autoDropTask.addStep(()->setLiftPosition(poleToPos[pole] - 190));
        autoDropTask.addStep(this::isLiftInTolerance);
        autoDropTask.addDelay(100); // reduce this delay as needed for tuning,was 500
        autoDropTask.addStep(()->setGrabberOpen(false));
        autoDropTask.addDelay(350); //TODO tune to less,was 2000
        autoDropTask.addStep(this::postAutoMove);
        autoDropTask.addStep(() -> triggerEvent(Events.dropComplete));
    }

    public void startAutoGrab(){
        if(getLiftPosition() < 900) {
            autoGrabTask.restart();
        }
    }

    public void addAutoGrabToTask(TaskEx task){
        task.addStep(this::startAutoGrab);
        task.waitForEvent(eventManager.getContainer(Events.grabComplete));
    }

    private void setMotorsToHomeConfig(){
        double power = -0.125;

        getHardware().leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getHardware().leftLiftMotor.setPower(power);
        //getHardware().rightLiftMotor.setPower(power);
    }
    private void setMotorsToRunConfig(){
        getHardware().leftLiftMotor.setPower(LifterHardware.liftHoldPower);
        //getHardware().rightLiftMotor.setPower(LifterHardware.liftHoldPower);

        getHardware().leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void constructAutoHome(){
        autoHomeTask.autoStart = false;


        autoGrabTask.addStep(() -> {
            LifterControl.flipOpen = 0;
            setGrabberClosed();
        });

        autoHomeTask.addStep(() -> {
            setMotorsToHomeConfig();
        });
        autoHomeTask.addTimedStep(() -> {
            parent.opMode.telemetry.addData("homing", getHardware().limitSwitch.getState());
        }, () -> !getHardware().limitSwitch.getState(), 5000);
        autoHomeTask.addStep(() -> {
            getHardware().leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getHardware().leftLiftMotor.setTargetPosition(0);
            //getHardware().rightLiftMotor.setTargetPosition(0);

            liftTargetPosition = 0;

            setMotorsToRunConfig();
        });
    }

    public void startAutoHome(){
        autoHomeTask.restart();
    }

    private void constructAutoDock(){
        autoDockTask.autoStart = false;

        autoDockTask.addStep(this::preAutoMove);
//        autoDockTask.addStep(() -> {          // idk what this is for
//            if(isLiftTurnSafe())
//                setGrabberOpen(false);
//            else {
//                LifterControl.flipOpen = 0;
//                setGrabberClosed();
//            }
//        });
        LifterControl.flipOpen = 0;
        setGrabberClosed();
        autoDockTask.addStep(()->setTurnPosition(.95));
        autoDockTask.addDelay(dockDelay);
        autoDockTask.addStep(()->setLiftPosition(coneToPos[cone]));
        autoDockTask.addStep(this::isLiftInTolerance);
        autoDockTask.addStep(()-> setGrabberOpen(false));
        autoDockTask.addStep(()-> {LifterControl.flipOpen = (cone == 0 ? 2 : 1);});
        autoDockTask.addStep(this::postAutoMove);
        autoDockTask.addStep(() -> triggerEvent(Events.dockComplete));
    }

    public void emergencyStop(){
        movementTask.runCommand(Group.Command.PAUSE);
        movementTask.getActiveRunnables().clear();

        setMotorsToRunConfig();

        setLiftPositionUnsafe(getLiftPosition());

        triggerEvent(ControllablePart.Events.startControllers); //TODO make better
    }

    /**
     * MUST RUN autoDock before
     */
    private void constructAutoGrab(){

        autoGrabTask.autoStart = false;

        autoGrabTask.addStep(this::preAutoMove);
        //autoGrabTask.addDelay(2000);
        autoGrabTask.addStep(() -> {
            LifterControl.flipOpen = 0;
            setGrabberClosed();
        });
        autoGrabTask.addDelay(200); // needed to let grabber close
        autoGrabTask.addStep(() -> setLiftPosition(coneToPos[cone] + 450));
        autoGrabTask.addStep(this::isLiftInTolerance);
        autoGrabTask.addStep(this::postAutoMove);
        autoGrabTask.addStep(() -> triggerEvent(Events.grabComplete));

    }

    // Line - shaped sensors
    public void doConeRange(DriveControl control) {
        int startDist = 30;
        double finalDist = 14;
        int tolerance = 1;
        int sideTol = 5;

//        double sidePower = 0.075;
        double sidePower = 0.08;
//        double forwardPower = 0.08;
//        double forwardP = 0.01;
        double forwardP = 0.015;
//        double sideShift = 1;
//        double farSideShift = 1.5;
//        double thisShift = sideShift;
        boolean inLeft = getLeftUltra() < startDist;
        boolean inRight = getRightUltra() < startDist;

        boolean inFarLeft = inLeft && (getLeftUltra() < getMidUltra());
        boolean inFarRight = inRight && (getRightUltra() < getMidUltra());

        double shortest = getShortest();


        if (getLiftPosition() > 500 && (shortest < startDist) && getCurrentTurnPosition() > 0.25) {
            if(startConeRange){
                if(positionSolver != null)
                    positionSolver.triggerEvent(Robot.Events.STOP);
                //code for when it is first in position
                startConeRange = false;
            }

//            if (inFarLeft || inFarRight) thisShift = farSideShift;
//            else thisShift = sideShift;

            if (inFarLeft) { // pole to the right
                control.power = control.power.addX(sidePower); // move to right
//                timesInPoleRange = 0;
            } else if (inFarRight) { // pole to the right
                control.power = control.power.addX(-sidePower); // move to left

//                positionSolver.addMoveToTaskEx(new Vector3(VectorMath.translateAsVector2(positionTracker.getCurrentPosition(), .5,0)));
//                timesInPoleRange = 0;
            }

            // looking for middle side to side
//            if (getMidUltra() > getLeftUltra() || getMidUltra() > getRightUltra()) { // only slide when needed
//                if (inLeft && !positionSolver.xRelativeChannel.isRunning()) { // pole to the right
//                    positionSolver.xRelativeChannel.setNewTarget(thisShift, true);
//                    //control.power = control.power.addX(sidePower); // move to right
////                timesInPoleRange = 0;
//                } else if (inRight && !positionSolver.xRelativeChannel.isRunning()) { // pole to the right
//                    positionSolver.xRelativeChannel.setNewTarget(-thisShift, true);
//                    //positionSolver.setNewTarget(VectorMath.translateAsVector2(positionTracker.getCurrentPosition(), -thisShift, 0), true);
//                    //control.power = control.power.addX(-sidePower); // move to left
////                positionSolver.addMoveToTaskEx(new Vector3(VectorMath.translateAsVector2(positionTracker.getCurrentPosition(), .5,0)));
////                timesInPoleRange = 0;
//                }
//            }
            // setting in/out range
//            if (shortest - tolerance > finalDist) {
//                control.power = control.power.addY(-forwardPower);
//                timesInPoleRange = 0;
//            } else if (shortest + tolerance < finalDist) {
//                control.power = control.power.addY(forwardPower);
//                timesInPoleRange = 0;
//            } else {
//                /* lined up in/out */
//                if(!(inLeft || inRight))
//                    timesInPoleRange++;
//            }

            control.power = control.power.addY(forwardP * (finalDist - shortest));

            if(Math.abs(finalDist - shortest) <= tolerance && !(inFarLeft || inFarRight)) {
                timesInPoleRange++;
//                if(predrop && !autoDropRun) {
//                    startAutoDrop();
//                    autoDropRun = true;
//                }
            }
            else {
                timesInPoleRange = Math.max(timesInPoleRange-1, 0);
                autoDropRun = false;
            }
        }
        else {
            /*Not in close enough to polish position yet */
//            if(!startConeRange)
//                if(positionSolver != null)
//                    positionSolver.triggerEvent(Robot.Events.START);
            startConeRange = true;

            timesInPoleRange = 0;
//            autoDropRun = false;
        }
    }


    public void doConeRangeTeleop(DriveControl control) {
        int startDist = 30;
        double finalDist = 14.5;
        int tolerance = 1;

        double sidePower = 0.08;
        double forwardP = 0.015;
        boolean inLeft = getLeftUltra() < startDist;
        boolean inRight = getRightUltra() < startDist;

        boolean inFarLeft = inLeft && (getLeftUltra() < getMidUltra());
        boolean inFarRight = inRight && (getRightUltra() < getMidUltra());

        double shortest = getShortest();

        if (getLiftPosition() > 500 && (shortest < startDist) && getCurrentTurnPosition() > 0.25) {
            if(startConeRange){
                if(positionSolver != null)
                    positionSolver.triggerEvent(Robot.Events.STOP);
                startConeRange = false;
            }

            if (inFarLeft) { // pole to the right
                control.power = control.power.addX(sidePower); // move to right
            } else if (inFarRight) { // pole to the right
                control.power = control.power.addX(-sidePower); // move to left
            }

            control.power = control.power.addY(forwardP * (finalDist - shortest));

            if(Math.abs(finalDist - shortest) <= tolerance && !(inFarLeft || inFarRight)) {
                timesInPoleRange++;
                if(predrop && !autoDropRun) {
                    startAutoDrop();
                    autoDropRun = true;
                }
            }
            else {
                timesInPoleRange = Math.max(timesInPoleRange-1, 0);
                autoDropRun = false;
            }
        }
        else {
            /*Not in close enough to polish position yet */
            startConeRange = true;

            timesInPoleRange = 0;
        }
    }

    @Override
    public void onInit() {
        //powerEdgeDetector.setOnFall(() -> se);
        constructAutoGrab();
        constructAutoDock();
        constructAutoDrop();
        constructAutoDrop2();
        constructAutoPreDrop();
        constructConeRanging();
        constructAutoHome();
        constructAutoMoveToCone();

        //add events
        eventManager.attachToEvent(Events.grabComplete, "decrement cone", () -> {setCone(cone - 1);});

        //set start position
        setTurnPosition(getSettings().turnServoStartPosition);
        setGrabberClosed();

        //homing
        homingEdge.setOnRise(() -> {
            getHardware().leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getHardware().leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        });

        //configure movement
        movementTask.forceActiveRunnablesDefault = false;
        movementTask.setMaxActiveRunnables(1);
    }

//    public void addAutoGrabPre(TimedTask task, int conePos){
//        task.addStep(() -> triggerEvent(ControllablePart.Events.stopControllers));
//        task.addStep(()-> setGrabberClosed());
//        task.addStep(()-> setLiftPosition(conePos + 400));
//        task.addStep(this::isLiftInTolerance);
//        task.addStep(()->setTurnPosition(.95));
////        task.addStep(() -> setGrabberOpen(true));
//        task.addStep(() -> triggerEvent(ControllablePart.Events.startControllers));
//    }

//    public void addAutoOpenGrabber(TimedTask task){
//        task.addStep(()-> setGrabberOpen(false));
//    }

//    public void addAutoGrabToTask(TimedTask task, int conePos){
//        task.addStep(() -> triggerEvent(Events.stopControllers));
//        task.addStep(() -> setGrabberClosed());
//        task.addStep(() -> setLiftPosition(conePos + 400));
//        task.addStep(this::isLiftInTolerance);
//        task.addStep(() -> setTurnPosition(0.95));
//        //task.addDelay(2000);
//        task.addStep(() -> setGrabberOpen(false));
//        task.addDelay(500); // needed to let grabber open
//        task.addStep(() -> setLiftPosition(conePos));
//        task.addStep(this::isLiftInTolerance);
//        //task.addDelay(2000);
//        task.addStep(() -> setGrabberClosed());
//        task.addDelay(500); // needed to let grabber close
//        task.addStep(() -> setLiftPosition(conePos + 400));
//        task.addStep(this::isLiftInTolerance);
//        task.addStep(() -> triggerEvent(Events.startControllers));
//    }

    public void constructConeRanging(){
        coneRangeingTask.addStep(() -> {
            getHardware().leftUltrasonic.measureRange();
            double thisDist = getHardware().midUltrasonic.getDistanceCm();
            midDist = (thisDist == -1) ? (150) : (thisDist); // when out of range report max
        });
        coneRangeingTask.addDelay(30);
        coneRangeingTask.addStep(() -> {
            getHardware().rightUltrasonic.measureRange();
            double thisDist = getHardware().leftUltrasonic.getDistanceCm();
            leftDist = (thisDist == -1) ? (150) : (thisDist); // when out of range report max
        });
        coneRangeingTask.addDelay(30);
        coneRangeingTask.addStep(() -> {
            getHardware().midUltrasonic.measureRange();
            double thisDist = getHardware().rightUltrasonic.getDistanceCm();
            rightDist = (thisDist == -1) ? (150) : (thisDist); // when out of range report max
        });
        coneRangeingTask.addDelay(30);
        coneRangeingTask.autoReset = true;
        coneRangeingTask.autoStart = true;
    }

    public double getShortest(){
        return Math.min(Math.min(getLeftUltra(), getRightUltra()), getMidUltra());
    }

    public boolean inTolerance(double first, double second, double tol){
        return Math.abs(first - second) <= tol;
    }

    public boolean isGreater(double first, double second, double tol){
        return first > second && first - second >= tol;
    }

    public boolean isPoleInRange(){
        return timesInPoleRange > 10;
    }

    @Override
    public void onBeanLoad() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false, true);
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, true);
    }

    @Override
    public void onRun(LifterControl control) { //TODO separate keeping lifter motor position from onRun
        liftWithPower(control.lifterPower, control.forceMoveLifter);
        turnWithPower(control.turningPower);
        //setGrabberPower(control.closePower);
        if(LifterControl.flipOpen == 0) //TODO fix this crazy logic
        {
            if(control.close)
                setGrabberClosed();
            else
                setGrabberOpen(false);
        } else {
            if (control.close)
                setGrabberOpen(LifterControl.flipOpen == 2);
            else
                setGrabberClosed();
        }

        coneDist = getHardware().coneSensor.getDistance(DistanceUnit.CM);

        homingEdge.accept(!getHardware().limitSwitch.getState());

        parent.opMode.telemetry.addData("Lifter height", getLiftPosition());
        parent.opMode.telemetry.addData("Lifter turn", getCurrentTurnPosition());
        parent.opMode.telemetry.addData("cone dist raw", coneDist);
        parent.opMode.telemetry.addData("cone dist", getDistToCone());
        parent.opMode.telemetry.addData("pole in range", isPoleInRange());
    }

    public static final class ContollerNames {
        public static final String distanceContoller = "distance controller"; //TODO make better
    }

    public static final class TaskNames{
        public final static String autoDock = "auto dock";
        public final static String autoGrab = "auto grab";
        public final static String preAutoDrop = "pre auto drop";
        public final static String coneMeasureRanges = "measure cone range";
        public final static String autoDrop = "auto drop";
        public final static String autoHome = "auto home";
        public final static String autoMoveToCone = "auto move to cone";
    }

    @Override
    public void onSettingsUpdate(LifterSettings lifterSettings) {}

    @Override
    public void onHardwareUpdate(LifterHardware lifterHardware) {

    }

    public static final class Events {
        public static final String dockComplete = "DOCK_COMPLETE";
        public static final String grabComplete = "gRAB_COMPLETE";
        public static final String preDropComplete = "PRE_DROP_COMPLETE";
        public static final String dropComplete = "DROP_COMPlETE";
    }

    @Override
    public void onStart() {
        LifterControl.flipOpen = 0;
        //taking out setturnpos to see if it messes up other steps in autograb
        // setTurnPosition(getSettings().turnServoStartPosition);
        //parent.opMode.getClass().getAnnotation(TeleOp.class);
        if (parent.opMode.getClass().getName().contains("TeleopForza")) {
            drive.addController(ContollerNames.distanceContoller, (control) -> doConeRangeTeleop(control));
        } else {
            drive.addController(ContollerNames.distanceContoller, (control) -> doConeRange(control));
        }
    }

    @Override
    public void onStop() {
        drive.removeController(ContollerNames.distanceContoller);
    }
}
