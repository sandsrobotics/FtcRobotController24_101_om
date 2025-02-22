package org.firstinspires.ftc.teamcode.parts.intake2;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake2Tasks {
    protected final Group movementTask;
    private final TimedTask autoHomeTask;
    private Intake2 intake;
    private final Robot robot;
    public final TimedTask autoBucketLiftTask;
    public final TimedTask autoBucketDropperTask;
    public final TimedTask getSpecimenTask;
    public final TimedTask hangSpecimenTask;
    public final TimedTask autoIntakeDropTask;
    public final TimedTask autoSamplePickupTask;
    public final TimedTask autoRotateServoSafe;
    public final TimedTask prepareToHangSpecimenTask;
    public final TimedTask autoSamplePickupTaskHack;
    public final TimedTask autoSpecimenHang1;
    public final TimedTask autoSpecimenHang2;

    public Intake2Tasks(Intake2 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        autoBucketLiftTask = new TimedTask(TaskNames.autoBucketLift, movementTask);
        autoBucketDropperTask = new TimedTask(TaskNames.autoBucketDropper, movementTask);
        getSpecimenTask = new TimedTask(TaskNames.autoSpecimenPickup, movementTask);
        hangSpecimenTask = new TimedTask(TaskNames.autoSpecimenHang, movementTask);
        autoSamplePickupTask = new TimedTask(TaskNames.autoSamplePickupTask, movementTask);
        autoIntakeDropTask = new TimedTask(TaskNames.autoIntakeDrop, movementTask);
        autoRotateServoSafe = new TimedTask(TaskNames.autoRotateServoSafe, movementTask);
        prepareToHangSpecimenTask = new TimedTask(TaskNames.autoSpecimenSet, movementTask);
        autoSamplePickupTaskHack = new TimedTask(TaskNames.autoSamplePickupTaskHack, movementTask);
        autoSpecimenHang1 = new TimedTask(TaskNames.autoSpecimenHang1, movementTask);
        autoSpecimenHang2 = new TimedTask(TaskNames.autoSpecimenHang2, movementTask);
    }

    public void constructAllIntakeTasks() {

    /* ***** autoHomeTask ******/
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(this::setSlideToHomeConfig);
        autoHomeTask.addStep(() -> {
            intake.incrementIntakeUpDown(0);
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperDockSafe);
            intake.getHardware().dropperServo.isDone();
            intake.getHardware().dropperServo.disable();
        });
        autoHomeTask.addTimedStep(() -> {
            robot.opMode.telemetry.addData("homing", intake.getHardware().bucketLiftZeroSwitch.getState());
        }, () -> intake.getHardware().bucketLiftZeroSwitch.getState(), 10000);
        autoHomeTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.getHardware().bucketLiftMotor.setTargetPosition(20);
            intake.bucketLiftTargetPosition = 20;
            setMotorsToRunConfig();
        });
    /* ***** prepareToHangSpecimenTask ******/
        prepareToHangSpecimenTask.autoStart = false;
        prepareToHangSpecimenTask.addStep(()-> intake.getHardware().dropperServo.disable());
        prepareToHangSpecimenTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        prepareToHangSpecimenTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        prepareToHangSpecimenTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenHangPosition,1));
        prepareToHangSpecimenTask.addStep(intake::isLiftInTolerance);

    /* ***** autoBucketLiftTask ******/
        autoBucketLiftTask.autoStart = false;
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        //Todo: maybe drive droper down if needed
        // autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperDockSafe);
        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.disable());
        autoBucketLiftTask.addStep(()-> intake.setLiftPosition(intake.getSettings().maxLiftPosition,1));
        autoBucketLiftTask.addStep(() -> intake.getHardware().bucketLiftMotor.getCurrentPosition() > 1100);
        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.enable());
        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin));
        autoBucketLiftTask.addStep(() -> intake.getHardware().bucketLiftMotor.getCurrentPosition() > 2600);
//        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.isDone()); //if greater than 500 set servo straight

    /* ***** autoBucketDropperTask ******/
        autoBucketDropperTask.autoStart = false;
        autoBucketDropperTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoBucketDropperTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.enable());
        // Todo: SAFETY make sure bucket is at/near top already
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMax));
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.isDone());
        autoBucketDropperTask.addDelay(350 ); // leave bucket high to dump sample
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperDockSafe));
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.isDone());
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.disable());
        if(intake.isTeleop) {
            autoBucketDropperTask.addStep(() -> intake.setLiftPosition(intake.getSettings().minLiftPosition, 1));
            autoBucketDropperTask.addStep(intake::isLiftInTolerance);
        }

    /* ***** getSpecimenTask ******/
        getSpecimenTask.autoStart = false;
        getSpecimenTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmStraightUp));
        getSpecimenTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        getSpecimenTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        getSpecimenTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        getSpecimenTask.addStep(()-> intake.getHardware().dropperServo.disable());
        getSpecimenTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenSafeHeight,1));
        getSpecimenTask.addStep(intake::isLiftInTolerance);

    /* ***** hangSpecimenTask ******/
        hangSpecimenTask.autoStart = false;
        hangSpecimenTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        hangSpecimenTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        hangSpecimenTask.addStep(()-> intake.getHardware().dropperServo.disable());
        hangSpecimenTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenServoOpenHeight,1));
        hangSpecimenTask.addStep(intake::isLiftInTolerance);
        hangSpecimenTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition));
        hangSpecimenTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        hangSpecimenTask.addStep(()-> intake.setLiftPosition(intake.getSettings().minLiftPosition,1));
        hangSpecimenTask.addStep(intake::isLiftInTolerance);

        /* ***** autoSpecimenHang1 ******/
        autoSpecimenHang1.autoStart = false;
        autoSpecimenHang1.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenServoOpenHeight,1));
        autoSpecimenHang1.addStep(intake::isLiftInTolerance);
        autoSpecimenHang1.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition));
        autoSpecimenHang1.addStep(()-> intake.getHardware().specimenServo.isDone());

        /* ***** autoSpecimenHang1 ******/
        autoSpecimenHang2.autoStart = false;
        autoSpecimenHang2.addStep(()-> intake.setLiftPosition(intake.getSettings().minLiftPosition,1));
        autoSpecimenHang2.addStep(intake::isLiftInTolerance);

    /* ***** autoIntakeDropTask ******/
        autoIntakeDropTask.autoStart = false;
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmAtBucket));
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoIntakeDropTask.addDelay(200); // because servo is loaded going up
        autoIntakeDropTask.addStep(()-> setIntakeWheels(0.0));
        autoIntakeDropTask.addDelay(100);  // transfer sample to bucket
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoIntakeDropTask.addStep(()-> setIntakeWheels(0.5));
        autoIntakeDropTask.addStep(()-> intake.getHardware().dropperServo.disable());

    /* ***** autoSamplePickupTask ******/
        autoSamplePickupTask.autoStart = false;
        autoSamplePickupTask.addStep(()-> setIntakeWheels(1.0)); //forward
        autoSamplePickupTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmAtSpecimen));
        autoSamplePickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTask.addStep(()-> intake.getHardware().rotationServo.setPosition(0.60));  // was .7
        autoSamplePickupTask.addStep(()-> intake.getHardware().rotationServo.isDone() || intake.readSampleDistance() < 0.7);
        autoSamplePickupTask.addStep(()-> intake.getHardware().rotationServo.setPosition(0.40));  // was .4
        autoSamplePickupTask.addStep(()-> intake.getHardware().rotationServo.isDone() || intake.readSampleDistance() < 0.7);
        autoSamplePickupTask.addStep(()-> intake.getHardware().rotationServo.setPosition(0.50));
        autoSamplePickupTask.addStep(()-> intake.getHardware().rotationServo.isDone());
        autoSamplePickupTask.addTimedStep(()-> intake.strafePower = 0.7,()->intake.readSampleDistance() < 0.7,300);
        autoSamplePickupTask.addTimedStep(()->intake.strafePower = -0.7,()->intake.readSampleDistance() < 0.7,600);
        autoSamplePickupTask.addStep(()-> setIntakeWheels(0.5));
        autoSamplePickupTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoSamplePickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTask.addStep(autoIntakeDropTask::restart);
        autoSamplePickupTask.addStep(autoIntakeDropTask::isDone);

        /* ***** autoRotateServoSafe ******/
        autoRotateServoSafe.autoStart = false;
        autoRotateServoSafe.addStep(()-> intake.getHardware().rotationServo.setPosition(intake.getSettings().intakeArmSafe));
        autoRotateServoSafe.addStep(()-> intake.getHardware().rotationServo.isDone());

        /* ***** autoSamplePickupTaskHack ******/
        autoSamplePickupTaskHack.autoStart = false;
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().rotationServo.setPosition(0.821));
        autoSamplePickupTaskHack.addStep(() -> intake.getHardware().rotationServo.isDone());
        autoSamplePickupTaskHack.addStep(()-> setIntakeWheels(1.0));
        autoSamplePickupTaskHack.addStep(()->intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmAtSpecimen));
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().tiltServo.isDone());
        //Todo: could optimize these delays in hack-task
        autoSamplePickupTaskHack.addDelay(300); // a bit of time to pickup sample //0.175
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().rotationServo.setPosition(0.483));
        autoSamplePickupTaskHack.addStep(() -> intake.getHardware().rotationServo.isDone());
        autoSamplePickupTaskHack.addDelay(250); // a bit of time to pickup sample //0.175
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().rotationServo.setPosition(0.725));
        autoSamplePickupTaskHack.addStep(() -> intake.getHardware().rotationServo.isDone());
        autoSamplePickupTaskHack.addDelay(250); // a bit of time to pickup sample //0.175
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().rotationServo.setPosition(0.5));
        autoSamplePickupTaskHack.addStep(() -> intake.getHardware().rotationServo.isDone());
//        autoSamplePickupTaskHack.addTimedStep(()-> intake.strafePower = 0.6,()->intake.readSampleDistance() < 0.7,400);
//        autoSamplePickupTaskHack.addTimedStep(()-> intake.strafePower = -0.6,()->intake.readSampleDistance() < 0.7,400);
        autoSamplePickupTaskHack.addStep(()-> setIntakeWheels(0.5));
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTaskHack.addStep(autoIntakeDropTask::restart);
        autoSamplePickupTaskHack.addStep(autoIntakeDropTask::isDone);
//        autoSamplePickupTaskHack.addD elay(200);
    }
    public void startAutoHome() {
        autoHomeTask.restart();
    }
    public void startAutoBucketLift() {autoBucketLiftTask.restart();}
    public void startAutoBucketDropper() {autoBucketDropperTask.restart();}
    public void startAutoSpecimenPickup() {getSpecimenTask.restart();}
    public void startAutoSpecimenHang() {hangSpecimenTask.restart(); }
    public void startAutoIntakeDropTask() {
        autoIntakeDropTask.restart();
    }
    public void startAutoSpecimenSet() {prepareToHangSpecimenTask.restart();}
    public void startAutoSamplePickup() {
        autoSamplePickupTask.restart();
    }
    public void startAutoSamplePickupHack() {
        autoSamplePickupTaskHack.restart();
    }
    public void startAutoRotateServoSafe() {
        autoRotateServoSafe.restart();
    }

    private void setSlideToHomeConfig() {
        double power = -0.3;
        intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.getHardware().bucketLiftMotor.setPower(power);
    }

    public void setMotorsToRunConfig() {
        intake.getHardware().bucketLiftMotor.setPower(IntakeHardware.slideHoldPower);
        intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setIntakeWheels(double speed) {
        intake.getHardware().intakeWheelServoLeft.setPosition(speed);
        intake.getHardware().intakeWheelServoRight.setPosition(speed);
    }
    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autoHome = "auto home";
        public final static String autoBucketLift = "auto bucket lift";
        public final static String autoBucketDropper = "auto bucket drop";
        public final static String autoSpecimenPickup = "auto specimen pickup";
        public final static String autoSpecimenHang = "auto specimen hang";
        public final static String autoIntakeDrop = "drop block into bucket";
        public final static String autoRotateServoSafe = "rotate servo to safe speed";
        public final static String autoSamplePickupTask = "auto sample pickup";
        public final static String autoSamplePickupTaskHack = "auto sample pickup hack";
        public final static String autoSpecimenSet = "auto specimen set";
        public final static String autoSpecimenHang1 = "auto specimen set1";
        public final static String autoSpecimenHang2 = "auto specimen set2";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
