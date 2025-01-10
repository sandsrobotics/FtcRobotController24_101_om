package org.firstinspires.ftc.teamcode.parts.intake2;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake2Tasks {
    private final Group movementTask;
    private final TimedTask autoHomeTask;
    private Intake2 intake;
    private Robot robot;
    private final TimedTask autoBucketLiftTask;
    private final TimedTask autoBucketDropperTask;
    private final TimedTask autoSpecimenPickupTask;
    private final TimedTask autoSpecimenHangTask;


    public Intake2Tasks(Intake2 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        autoBucketLiftTask = new TimedTask(TaskNames.autoBucketLift, movementTask);
        autoBucketDropperTask = new TimedTask(TaskNames.autoBucketDropper, movementTask);
        autoSpecimenPickupTask = new TimedTask(TaskNames.autoSpecimenPickup, movementTask);
        autoSpecimenHangTask = new TimedTask(TaskNames.autoSpecimenHang, movementTask);
    }

    public void constructAutoHome() {
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(this::setSlideToHomeConfig);
        autoHomeTask.addStep(() -> {
            intake.incrementIntakeUpDown(0);
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
            intake.getHardware().dropperServo.getController().pwmDisable();
        });
        autoHomeTask.addTimedStep(() -> {
            robot.opMode.telemetry.addData("homing", intake.getHardware().bucketLiftZeroSwitch.getState());
        }, () -> intake.getHardware().bucketLiftZeroSwitch.getState(), 10000);
        autoHomeTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.getHardware().bucketLiftMotor.setTargetPosition(0);
            intake.slideTargetPosition = 0;
            setMotorsToRunConfig();
        });
    }
    public void constructAutoBucketLift() {
        autoBucketLiftTask.autoStart = false;
        autoBucketLiftTask.addStep(() -> {
            intake.incrementIntakeUpDown(2007); //CHANGE NUMBER. NUMBER IF STATEMENT IS SET IN INCREMENTINTAKEUPDOWN
        });
        autoBucketLiftTask.addDelay(1500);
        autoBucketLiftTask.addStep(() -> {
            intake.incrementIntakeUpDown(0);
            intake.getHardware().dropperServo.getController().pwmDisable();
        });
        autoBucketLiftTask.addStep( ()-> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().maxLiftPosition);
        }, () -> intake.getHardware().bucketLiftMotor.getCurrentPosition() > 600);
        autoBucketLiftTask.addStep( ()->{
            intake.getHardware().dropperServo.getController().pwmEnable();
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin);
        });
    }

    public void constructAutoBucketDropper() {
        autoBucketDropperTask.autoStart = false;
        autoBucketDropperTask.addStep(() -> {
            intake.getHardware().dropperServo.getController().pwmEnable();
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMax);
        });
        autoBucketDropperTask.addDelay(1500);
        autoBucketDropperTask.addStep( ()-> {
            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin);
        }, () -> intake.getHardware().dropperServo.getPosition() == intake.getSettings().dropperServoMin);
        autoBucketDropperTask.addDelay(500);
        autoBucketDropperTask.addStep(() -> {
            intake.getHardware().dropperServo.getController().pwmDisable();
        });
        autoBucketDropperTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().minLiftPosition);
        });
    }

    public void constructAutoSpecimenPickup() {
        autoSpecimenPickupTask.autoStart = false;
        autoSpecimenPickupTask.addStep(() -> {
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition);
        });
        autoSpecimenPickupTask.addDelay(1000);
        autoSpecimenPickupTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().specimenSafeHeight);
        });
    }
    public void constructAutoSpecimenHang() {
        autoSpecimenHangTask.autoStart = false;
        Vector3 currentpos = intake.pt.getCurrentPosition();
        autoSpecimenHangTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().specimenHangPosition);
        });

        autoSpecimenHangTask.addDelay(1000);

        autoSpecimenHangTask.addStep(() -> {
            currentpos.addY(-2);
            intake.drive.moveRobot(currentpos);
        });

        autoSpecimenHangTask.addDelay(1000);

        autoSpecimenHangTask.addStep( ()-> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().specimenServoOpenHeight);
            intake.slideTargetPosition = intake.getSettings().specimenServoOpenHeight;
        }, () -> intake.isLiftInTolerance());

        autoSpecimenHangTask.addStep(() -> {
            intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoOpenPosition);
        });

        autoSpecimenHangTask.addDelay(1000);

        autoSpecimenHangTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setTargetPosition(0);
        });
    }

    public void startAutoHome() {
        autoHomeTask.restart();
    }

    public void startAutoBucketLift() {
        autoBucketLiftTask.restart();
    }

    public void startAutoBucketDropper() {
        autoBucketDropperTask.restart();
    }

    public void startAutoSpecimenPickup() {
        autoSpecimenPickupTask.restart();
    }
    public void startAutoSpecimenHang() {
        autoSpecimenHangTask.restart();
    }

    private void setSlideToHomeConfig() {
        double power = -0.125;
        intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.getHardware().bucketLiftMotor.setPower(power);
    }

    private void setMotorsToRunConfig() {
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
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
