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
    public final TimedTask autoSpecimenPickupTask;
    public final TimedTask autoSpecimenHangTask;
    public final TimedTask autoIntakeDropTask;
    public final TimedTask autoSamplePickupTask;
    public final TimedTask autoRotateServoSafe;
    public final TimedTask autoSpecimenSetTask;
    public final TimedTask autoSamplePickupTaskHack;

    public Intake2Tasks(Intake2 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        autoBucketLiftTask = new TimedTask(TaskNames.autoBucketLift, movementTask);
        autoBucketDropperTask = new TimedTask(TaskNames.autoBucketDropper, movementTask);
        autoSpecimenPickupTask = new TimedTask(TaskNames.autoSpecimenPickup, movementTask);
        autoSpecimenHangTask = new TimedTask(TaskNames.autoSpecimenHang, movementTask);
        autoSamplePickupTask = new TimedTask(TaskNames.autoSamplePickupTask, movementTask);
        autoIntakeDropTask = new TimedTask(TaskNames.autoIntakeDrop, movementTask);
        autoRotateServoSafe = new TimedTask(TaskNames.autoRotateServoSafe, movementTask);
        autoSpecimenSetTask = new TimedTask(TaskNames.autoSpecimenSet, movementTask);
        autoSamplePickupTaskHack = new TimedTask(TaskNames.autoSamplePickupTaskHack, movementTask);
    }

    public void constructAllIntakeTasks() {

    /* ***** autoHomeTask ******/
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(this::setSlideToHomeConfig);
        autoHomeTask.addStep(() -> {
            intake.incrementIntakeUpDown(0);
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
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
    /* ***** autoSpecimenHang ******/
        autoSpecimenSetTask.autoStart = false;
        autoSpecimenSetTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenHangPosition,1));
        autoSpecimenSetTask.addStep(intake::isLiftInTolerance);

    /* ***** autoBucketLiftTask ******/
        autoBucketLiftTask.autoStart = false;
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        //Todo: lift to a safe mid height then start the dropperServo Min below so it will be ready when the lift makes it to the top
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
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMax));
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.isDone());
        autoBucketDropperTask.addDelay(220); // leave bucket high to dump sample
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin));
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.isDone());
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.disable());
        if(intake.isTeleop) {
            autoBucketDropperTask.addStep(() -> intake.setLiftPosition(intake.getSettings().minLiftPosition, 1));
            autoBucketDropperTask.addStep(intake::isLiftInTolerance);
        }

    /* ***** autoSpecimenPickupTask ******/
        autoSpecimenPickupTask.autoStart = false;
        //todo: autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.stop());
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmStraightUp));
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        autoSpecimenPickupTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenSafeHeight,1));
        autoSpecimenPickupTask.addStep(intake::isLiftInTolerance);

    /* ***** autoSpecimenHangTask ******/
        autoSpecimenHangTask.autoStart = false;
        //todo: autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.stop());autoSpecimenPickupTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmStraightUp));
        //        autoSpecimenPickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSpecimenHangTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenServoOpenHeight,1));
        autoSpecimenHangTask.addStep(intake::isLiftInTolerance);
        autoSpecimenHangTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition));
        autoSpecimenHangTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        autoSpecimenHangTask.addStep(()-> intake.setLiftPosition(intake.getSettings().minLiftPosition,1));
        autoSpecimenHangTask.addStep(intake::isLiftInTolerance);

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

        autoSamplePickupTask.addStep(() -> intake.getHardware().rotationServo.setPosition(0.60));  // was .7
        autoSamplePickupTask.addStep(() -> intake.getHardware().rotationServo.isDone() || intake.readSampleDistance() < 0.7);

        autoSamplePickupTask.addStep(() -> intake.getHardware().rotationServo.setPosition(0.40));  // was .4
        autoSamplePickupTask.addStep(() -> intake.getHardware().rotationServo.isDone() || intake.readSampleDistance() < 0.7);

        autoSamplePickupTask.addStep(() -> intake.getHardware().rotationServo.setPosition(0.50));
        autoSamplePickupTask.addStep(() -> intake.getHardware().rotationServo.isDone());

//        autoSamplePickupTask.addStep(() -> intake.drive.moveRobot(intake.pt.getCurrentPosition().addZ(0.5)));
//        autoSamplePickupTask.addDelay(100);
//        autoSamplePickupTask.addStep(() -> intake.drive.moveRobot(intake.pt.getCurrentPosition().addZ(-0.5)));
//        autoSamplePickupTask.addDelay(100);

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
        //Todo: Add left and right rotation like in non-hack task
        autoSamplePickupTaskHack.autoStart = false;
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().rotationServo.setPosition(0.821));
        autoSamplePickupTaskHack.addStep(() -> intake.getHardware().rotationServo.isDone());
        autoSamplePickupTaskHack.addStep(()-> setIntakeWheels(1.0));
        autoSamplePickupTaskHack.addStep(()->intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmAtSpecimen));
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTaskHack.addDelay(200); // a bit of time to pickup sample //0.175
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().rotationServo.setPosition(0.483));
        autoSamplePickupTaskHack.addStep(()-> setIntakeWheels(0.5));
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoSamplePickupTaskHack.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTaskHack.addStep(autoIntakeDropTask::restart);
        autoSamplePickupTaskHack.addStep(autoIntakeDropTask::isDone);
        autoSamplePickupTaskHack.addDelay(200);
    }
    public void startAutoHome() {
        autoHomeTask.restart();
    }
    public void startAutoBucketLift() {autoBucketLiftTask.restart();}
    public void startAutoBucketDropper() {
        autoBucketDropperTask.restart();
    }
    public void startAutoSpecimenPickup() {
        autoSpecimenPickupTask.restart();
    }
    public void startAutoSpecimenHang() {autoSpecimenHangTask.restart(); }
    public void startAutoIntakeDropTask() {
        autoIntakeDropTask.restart();
    }
    public void startAutoSpecimenSet() {autoSpecimenSetTask.restart();}
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
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
