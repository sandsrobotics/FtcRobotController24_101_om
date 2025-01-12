package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake2.Intake2;
import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class IntakeTasks {
    private final Group movementTask;
    private final TimedTask autoHomeTask;
    private final TimedTask prepareToIntakeTask;
    private final TimedTask safeTask;
    private final TimedTask transferTask;
    private final TimedTask hangSpecimenTask;
    private final TimedTask prepareToHangSpecimenTask;
    private final TimedTask prepareToDepositTask;
    private final TimedTask depositTask;
    private final TimedTask autoIntakeTask;
    private final TimedTask prepareToTransferTask;
    private final TimedTask checkSampleTask;
    private final TimedTask ejectBadSample;
    private Intake intake;
    private Robot robot;

    public IntakeTasks(Intake intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        prepareToIntakeTask = new TimedTask(TaskNames.prepareToIntake, movementTask);
        safeTask = new TimedTask(TaskNames.safe, movementTask);
        transferTask = new TimedTask(TaskNames.transfer, movementTask);
        hangSpecimenTask = new TimedTask(TaskNames.hangSpecimen, movementTask);
        prepareToHangSpecimenTask = new TimedTask(TaskNames.prepareToHangSpecimen, movementTask);
        prepareToDepositTask = new TimedTask(TaskNames.prepareToDeposit, movementTask);
        depositTask = new TimedTask(TaskNames.deposit, movementTask);
        autoIntakeTask = new TimedTask(TaskNames.autoIntake, movementTask);
        prepareToTransferTask = new TimedTask(TaskNames.prepareToTransfer, movementTask);
        checkSampleTask = new TimedTask(TaskNames.checkSample, movementTask);
        ejectBadSample = new TimedTask(TaskNames.ejectBadSample, movementTask);
    }

    public void constructPrepareToIntakeTask() {
        prepareToIntakeTask.autoStart = false;
        // todo: kill other related tasks
        prepareToIntakeTask.addStep(() -> {
                    safeTask.reset();
                    intake.getHardware().flipper.setPosition(intake.getSettings().spintakeSafe);
                    intake.getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intake.getHardware().horizSliderMotor.setTargetPosition(intake.getSettings().positionSlideStartIntake);
                    intake.slideTargetPosition = intake.getSettings().positionSlideStartIntake;
                    intake.getHardware().horizSliderMotor.setPower(1);
                }, () ->
                    intake.isSlideInTolerance()
        );
        prepareToIntakeTask.addStep(() -> {
                    intake.getHardware().flipper.setPosition(intake.getSettings().spintakeAlmostFloor);
                }, () ->
                    intake.getHardware().flipper.isDone()
        );
    }

    public void constructSafeTask() {
        safeTask.autoStart = false;
        safeTask.addStep( () -> {
            prepareToIntakeTask.reset();
            intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen);
            intake.getHardware().flipper.setPosition(intake.getSettings().spintakeParked);
            intake.getHardware().chute.setPosition(intake.getSettings().chuteParked);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
            intake.getHardware().horizSliderMotor.setTargetPosition(intake.getSettings().positionSlideMin);
            intake.slideTargetPosition = intake.getSettings().positionSlideMin;
            intake.getHardware().horizSliderMotor.setPower(1);
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftMin);
            intake.liftTargetPosition = intake.getSettings().positionLiftMin;
            intake.getHardware().bucketLiftMotor.setPower(1);
        }, () ->
            intake.isSlideInTolerance() &&
                    intake.isLiftInTolerance() &&
                    intake.getHardware().flipper.isDone() &&
                    intake.getHardware().chute.isDone()
        );
        safeTask.addStep( () -> {
            intake.getHardware().flipper.disable();
            intake.getHardware().chute.disable();
                });
    };

    public void constructTransfer() {
        transferTask.autoStart = false;
        transferTask.addStep( () -> {
            safeTask.restart();
            }, ()->
            safeTask.isDone()
        );
        transferTask.addStep( () -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().spintakeParked);
            intake.getHardware().chute.setPosition(intake.getSettings().chuteParked);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOut);
        });
        transferTask.addDelay(1500);
        transferTask.addStep( () -> {
            intake.getHardware().flipper.disable();
            intake.getHardware().chute.disable();
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
        });
    }
    public void constructPrepareToDepositTask() {
        prepareToDepositTask.autoStart = false;
        prepareToDepositTask.addStep( () -> {
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
        }, ()->
             intake.getHardware().spinner.isDone()
                );
        prepareToDepositTask.addStep( () -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftReady);
            intake.liftTargetPosition = intake.getSettings().positionLiftReady;
            intake.getHardware().bucketLiftMotor.setPower(1);
        }, ()->
                intake.isLiftInTolerance()
        );
    }
    public void constructDepositTask() {
        depositTask.autoStart = false;
        depositTask.addStep( () -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftMax);
            intake.liftTargetPosition = intake.getSettings().positionLiftMax;
            intake.getHardware().bucketLiftMotor.setPower(1);
        }, ()->intake.isLiftInTolerance() && intake.getHardware().chute.isDone()
        );
        depositTask.addStep( ()->{
            intake.getHardware().chute.setPosition(intake.getSettings().chuteDeposit);
        }, ()->intake.getHardware().chute.isDone()
        );
        depositTask.addDelay(500);
        depositTask.addStep( () -> {
            safeTask.restart();
        });
    }
    public void constructAutoIntakeTask() {
        autoIntakeTask.autoStart = false;
        autoIntakeTask.addStep( ()->{
            intake.getHardware().flipper.setPosition((intake.getSettings().spintakeAlmostFloor));
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerIn);
            //TODO: FINISH THIS TASSKSKKKKSKSK
        });
    }




//        autoBucketLiftTask.addStep( ()-> {
//            intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().maxLiftPosition);
//        }, () -> intake.getHardware().bucketLiftMotor.getCurrentPosition() > 600);
                //setSpintakeServo(spintakeSafe)
        //intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMin);
//        autoBucketDropperTask.addStep(() -> {
//            intake.getHardware().dropperServo.getController().pwmEnable();
//            intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperServoMax);
//        });

    public void constructAutoHome() {
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(this::setSlideToHomeConfig);
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

    public void startAutoHome() {
        autoHomeTask.restart();
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

//    public static boolean isSlideInTolerance(int pos) {return Math.abs(motorSlide.getCurrentPosition() - pos) < toleranceSlide;}
//    public static boolean isSlideInTolerance() {return isSlideInTolerance(slideTargetPosition);}
//    public static boolean isLiftInTolerance(int pos) {return Math.abs(motorLift.getCurrentPosition() - pos) < toleranceLift;}
//    public static boolean isLiftInTolerance() {return isLiftInTolerance(liftTargetPosition);}

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autoHome = "auto home";
        public final static String prepareToIntake = "prepare to intake";
        public final static String safe = "safe";
        public final static String transfer = "transfer";
        public final static String hangSpecimen = "hang specimen";
        public final static String prepareToHangSpecimen = "prepare to hang specimen";
        public final static String prepareToDeposit = "prepare to deposit";
        public final static String deposit = "deposit";
        public final static String autoIntake = "auto intake";
        public final static String checkSample = "check sample";
        public final static String ejectBadSample = "eject the sample";
        public final static String prepareToTransfer = "prepare the transfer";
//        public final static String
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
