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
    }

    public void constructAllIntakeTasks() {

    /* ***** autoHomeTask ******/
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(this::setSlideToHomeConfig);
        autoHomeTask.addStep(() -> {
            intake.incrementIntakeUpDown(0);
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
            intake.getHardware().dropperServo.stop();
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

    /* ***** autoBucketLiftTask ******/
        autoBucketLiftTask.autoStart = false;
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.isDone() );
        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.stop());
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoBucketLiftTask.addStep(()-> intake.getHardware().tiltServo.isDone() );
        autoBucketLiftTask.addStep(()-> intake.setLiftPosition(intake.getSettings().maxLiftPosition,1));
        autoBucketLiftTask.addStep(intake::isLiftInTolerance);
        autoBucketLiftTask.addStep(()-> {
            intake.getHardware().dropperServo.enable();
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin);
        });
        autoBucketLiftTask.addStep(()-> intake.getHardware().dropperServo.isDone());

    /* ***** autoBucketDropperTask ******/
        autoBucketDropperTask.autoStart = false;
        autoBucketDropperTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoBucketDropperTask.addStep(()-> intake.getHardware().tiltServo.isDone() );
        autoBucketDropperTask.addStep(()-> {
            intake.getHardware().dropperServo.enable();
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMax);
        });
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.isDone() );
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin));
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.isDone() );
        autoBucketDropperTask.addStep(()-> intake.getHardware().dropperServo.stop());
        autoBucketDropperTask.addStep(()-> intake.setLiftPosition(intake.getSettings().minLiftPosition,1));
        autoBucketDropperTask.addStep(intake::isLiftInTolerance);

    /* ***** autoSpecimenPickupTask ******/
        autoSpecimenPickupTask.autoStart = false;
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmStraightUp));
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        autoSpecimenPickupTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        autoSpecimenPickupTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenSafeHeight,1));
        autoSpecimenPickupTask.addStep(intake::isLiftInTolerance);

    /* ***** autoSpecimenHangTask ******/
        autoSpecimenHangTask.autoStart = false;
        autoSpecimenHangTask.addStep(()-> intake.setLiftPosition(intake.getSettings().specimenServoOpenHeight,.7));
        autoSpecimenHangTask.addStep(intake::isLiftInTolerance);
        autoSpecimenHangTask.addStep(()-> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition));
        autoSpecimenHangTask.addStep(()-> intake.getHardware().specimenServo.isDone());
        autoSpecimenHangTask.addStep(()-> intake.setLiftPosition(intake.getSettings().minLiftPosition,1));
        autoSpecimenHangTask.addStep(intake::isLiftInTolerance);

    /* ***** autoIntakeDropTask ******/
        autoIntakeDropTask.autoStart = false;
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmAtBucket));
//        autoIntakeDropTask.addStep( ()-> intake.getHardware().tiltServo.isDone());
        autoIntakeDropTask.addStep(()-> {
            intake.getHardware().intakeWheelServoLeft.setPosition(0.0);
            intake.getHardware().intakeWheelServoRight.setPosition(0.0);
        });
        autoIntakeDropTask.addDelay(3000);  // transfer sample to bucket
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe));
        autoIntakeDropTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoIntakeDropTask.addStep(()-> {
            intake.getHardware().intakeWheelServoLeft.setPosition(0.5);
            intake.getHardware().intakeWheelServoRight.setPosition(0.5);
        });
        autoIntakeDropTask.addStep(()-> intake.getHardware().dropperServo.stop());

    /* ***** autoSamplePickupTask ******/
        autoSamplePickupTask.autoStart = false;
        autoSamplePickupTask.addStep(()-> {
            intake.getHardware().intakeWheelServoLeft.setPosition(1.0);
            intake.getHardware().intakeWheelServoRight.setPosition(1.0);
        });
        autoSamplePickupTask.addStep(()->intake.getHardware().tiltServo.setPosition(0.2));
        autoSamplePickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTask.addDelay(250); // needed?
        autoSamplePickupTask.addStep(()-> {
            intake.getHardware().intakeWheelServoLeft.setPosition(0.5);
            intake.getHardware().intakeWheelServoRight.setPosition(0.5);
            intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe);
        });
        autoSamplePickupTask.addStep(()-> intake.getHardware().tiltServo.isDone());
        autoSamplePickupTask.addStep(autoIntakeDropTask::restart);

        /* ***** autoRotateServoSafe ******/
        autoRotateServoSafe.autoStart = false;
        autoRotateServoSafe.addStep(()-> intake.getHardware().rotationServo.setPosition(intake.getSettings().intakeArmSafe));
        autoRotateServoSafe.addStep(()-> intake.getHardware().rotationServo.isDone());
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
    public void startAutoSamplePickup() {
        autoSamplePickupTask.restart();
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
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
