package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class IntakeTasks {
    public final Group movementTask;
    public final TimedTask autonomousSample;
    public final TimedTask autoHomeTask;
    public final TimedTask prepareToIntakeTask;
    public final TimedTask safeTask;
    public final TimedTask transferTask;
    public final TimedTask prepareToLowDumpIntake;
    public final TimedTask lowDumpIntake;
    public final TimedTask prepareToGetSpecimen;
    public final TimedTask getSpecimen;
    public final TimedTask hangSpecimenTask;
    public final TimedTask prepareToHangSpecimenTask;
    public final TimedTask prepareToDepositTask;
    public final TimedTask depositTask;
    public final TimedTask autoIntakeTask;
    public final TimedTask prepareToTransferTask;
    public final TimedTask checkSampleTask;
    public final TimedTask ejectBadSampleTask;
    public final TimedTask prepareToHangRobotTask;
    public final TimedTask hangRobotTask;
    private final Intake intake;
    private final Robot robot;

    public IntakeTasks(Intake intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autonomousSample = new TimedTask(TaskNames.autonomousSample, movementTask);
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
        prepareToIntakeTask = new TimedTask(TaskNames.prepareToIntake, movementTask);
        safeTask = new TimedTask(TaskNames.safe, movementTask);
        transferTask = new TimedTask(TaskNames.transfer, movementTask);
        prepareToLowDumpIntake = new TimedTask(TaskNames.prepareToLowDumpIntake, movementTask);
        lowDumpIntake = new TimedTask(TaskNames.lowDumpIntake, movementTask);
        prepareToGetSpecimen = new TimedTask(TaskNames.prepareToGetSpecimen, movementTask);
        getSpecimen = new TimedTask(TaskNames.getSpecimen, movementTask);
        hangSpecimenTask = new TimedTask(TaskNames.hangSpecimen, movementTask);
        prepareToHangSpecimenTask = new TimedTask(TaskNames.prepareToHangSpecimen, movementTask);
        prepareToDepositTask = new TimedTask(TaskNames.prepareToDeposit, movementTask);
        depositTask = new TimedTask(TaskNames.deposit, movementTask);
        autoIntakeTask = new TimedTask(TaskNames.autoIntake, movementTask);
        prepareToTransferTask = new TimedTask(TaskNames.prepareToTransfer, movementTask);
        checkSampleTask = new TimedTask(TaskNames.checkSample, movementTask);
        ejectBadSampleTask = new TimedTask(TaskNames.ejectBadSample, movementTask);
        prepareToHangRobotTask = new TimedTask(TaskNames.prepareToHangRobotTask, movementTask);
        hangRobotTask = new TimedTask(TaskNames.hangRobotTask, movementTask);
    }

    public void startAutoHome() { autoHomeTask.restart(); }

//    public void constructPrepareToIntakeTask() {
    public void constructAllIntakeTasks() {
        autonomousSample.autoStart = false;
        autonomousSample.addStep(() -> {
                    intake.getHardware().flipper.setPosition(intake.getSettings().spintakeAlmostFloor);
                    intake.getHardware().spinner.setPosition(intake.getSettings().spinnerIn);
        });
        autonomousSample.addStep(() -> intake.getHardware().flipper.isDone());
        autonomousSample.addStep(() -> intake.setSlidePosition(intake.getSettings().autoSampleSlideDistance, 1));
        autonomousSample.addStep(intake::isSlideInTolerance);
        autonomousSample.addDelay(500);
        autonomousSample.addStep(() -> intake.setSlidePosition(intake.getSettings().minSlidePosition, 1));
        autonomousSample.addStep(intake::isSlideInTolerance);
        autonomousSample.addStep(prepareToTransferTask::restart);
        autonomousSample.addStep(transferTask::isDone);
        /* todo: This needs more wiggling!!!! */

        prepareToIntakeTask.autoStart = false;
        prepareToIntakeTask.addStep(() -> {
            //safeTask.runCommand(Group.Command.PAUSE);
            intake.getHardware().flipper.setPosition(intake.getSettings().spintakeSafe);
            intake.setSlidePosition(intake.getSettings().positionSlideStartIntake, 1);
        });
        prepareToIntakeTask.addStep(intake::isSlideInTolerance);
        prepareToIntakeTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().spintakeAlmostFloor);
        });
        prepareToIntakeTask.addStep( () -> intake.getHardware().flipper.isDone() );
//    }

        prepareToGetSpecimen.autoStart = false;
        prepareToGetSpecimen.addStep(() -> {
            //safeTask.runCommand(Group.Command.PAUSE);
            //intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftGetSpecimen);
            intake.setLiftPosition(intake.getSettings().positionLiftGetSpecimen, 1);
            intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen);

        });
        prepareToGetSpecimen.addStep(intake::isLiftInTolerance);
        prepareToGetSpecimen.addStep(() -> intake.getHardware().pinch.isDone());

//    }
        getSpecimen.autoStart = false;
        getSpecimen.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchClosed));
        getSpecimen.addStep(() -> intake.getHardware().pinch.isDone());
        //getSpecimen.addStep(() -> intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftRaiseSpeciman));
        getSpecimen.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftRaiseSpeciman, 0.7));
        getSpecimen.addStep(intake::isLiftInTolerance);

        prepareToHangSpecimenTask.autoStart = false;
        prepareToHangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchLoose));
        //prepareToHangSpecimenTask.addStep(() -> intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftHangReady));
        prepareToHangSpecimenTask.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));
        prepareToHangSpecimenTask.addStep(intake::isLiftInTolerance);

        hangSpecimenTask.autoStart = false;
        hangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchSuperLoose));
        //hangSpecimenTask.addStep(() -> intake.getHardware().bucketLiftMotor.setTargetPosition(intake.getSettings().positionLiftHangRelease));
        hangSpecimenTask.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangRelease, 0.7));
        hangSpecimenTask.addStep(intake::isLiftInTolerance);
        hangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen));
        hangSpecimenTask.addStep(safeTask::restart);

    //    prepareToLowDumpIntake.autoStart = false;
      //  prepareToLowDumpIntake.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteParked));

        lowDumpIntake.autoStart = false;
        lowDumpIntake.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteDeposit));
        lowDumpIntake.addStep( ()-> intake.getHardware().chute.isDone());
        lowDumpIntake.addDelay(500);
        lowDumpIntake.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteParked));

        prepareToHangRobotTask.autoStart = false;
        //prepareToHangRobotTask.addStep(() -> intake.getHardware().robotHangMotor.setTargetPosition(intake.getSettings().positionHangReady));
        prepareToHangRobotTask.addStep(() -> intake.setHangPosition(intake.getSettings().positionHangReady, 1));

        hangRobotTask.autoStart = false;
        //hangRobotTask.addStep(() -> intake.getHardware().robotHangMotor.setTargetPosition(intake.getSettings().positionHangFinal));
        hangRobotTask.addStep(() -> intake.setHangPosition(intake.getSettings().positionHangFinal, 1));


//    public void constructSafeTask() {
        safeTask.autoStart = false;
        safeTask.addStep( () -> {
            //prepareToIntakeTask.runCommand(Group.Command.PAUSE);
            intake.preventUserControl = true;
            intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen);
            intake.getHardware().flipper.setPosition(intake.getSettings().spintakeParked);
            intake.getHardware().chute.setPosition(intake.getSettings().chuteParked);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
            intake.setSlidePosition(intake.getSettings().positionSlideOvershoot, 1);
            intake.setLiftPosition(intake.getSettings().positionLiftMin, 1);
            intake.getHardware().park.setPosition(intake.getSettings().parkDown);
        });
        safeTask.addStep( () ->
            intake.isSlideInTolerance() &&
            intake.isLiftInTolerance() &&
            intake.getHardware().flipper.isDone() &&
            intake.getHardware().chute.isDone());
        safeTask.addStep( () -> {
            intake.getHardware().flipper.disable();
            intake.getHardware().chute.disable();
            intake.preventUserControl = false;
        });
//    }

//    public void constructTransfer() {
        transferTask.autoStart = false;
        transferTask.addStep(safeTask::restart);
        transferTask.addStep(safeTask::isDone);
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
//    }


//    public void constructPrepareToDepositTask() {
        prepareToDepositTask.autoStart = false;
        prepareToDepositTask.addStep( () -> {
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
        });
        prepareToDepositTask.addStep( ()-> intake.getHardware().spinner.isDone());
        prepareToDepositTask.addStep( () -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.setLiftPosition(intake.getSettings().positionLiftReady, 1);
        });
        prepareToDepositTask.addStep(intake::isLiftInTolerance);
//    }

//    public void constructDepositTask() {
        depositTask.autoStart = false;
        depositTask.addStep( () -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.setLiftPosition(intake.getSettings().positionLiftMax, 1);
        });
        depositTask.addStep( ()->intake.isLiftInTolerance() && intake.getHardware().chute.isDone());
        depositTask.addStep( ()->{
            intake.getHardware().chute.setPosition(intake.getSettings().chuteDeposit);
        });
        depositTask.addStep( ()-> intake.getHardware().chute.isDone());
        depositTask.addDelay(500);
        depositTask.addStep(safeTask::restart);
//    }

//    public void constructAutoIntakeTask() {
        autoIntakeTask.autoStart = false;
        autoIntakeTask.addStep( ()->{
            intake.getHardware().flipper.setPosition((intake.getSettings().spintakeAlmostFloor));
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerIn);
        });
        //lk added this; subtract if trouble
        autoIntakeTask.addStep(()-> intake.getHardware().flipper.isDone());
        autoIntakeTask.addStep(()-> intake.getHardware().flipper.disable());
        //end lk
        autoIntakeTask.addStep(()-> (intake.sampleDistance() < 1.5));
        autoIntakeTask.addStep(()->(intake.identifySampleColor() > 0));
        autoIntakeTask.addStep(()->{
           if (intake.isSampleGood(intake.lastSample)) {
                prepareToTransferTask.restart();
           } else {
               ejectBadSampleTask.restart();
           }
        });

        ejectBadSampleTask.autoStart = false;
        ejectBadSampleTask.addStep( ()-> {
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOut);

        });
        ejectBadSampleTask.addDelay(1500);
        ejectBadSampleTask.addStep(autoIntakeTask::restart);

        prepareToTransferTask.autoStart = false;
        prepareToTransferTask.addStep( ()-> {
           intake.getHardware().flipper.setPosition(intake.getSettings().spintakeSafe);
           intake.getHardware().spinner.setPosition(intake.getSettings().spinnerSlowOut);
        });
        prepareToTransferTask.addDelay(350);
        prepareToTransferTask.addStep( ()->{
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
            transferTask.restart();
        });


//    }

//    public void constructAutoHome() {
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(()->this.setSlideToHomeConfig(1));
        autoHomeTask.addTimedStep(() -> {
            robot.opMode.telemetry.addData("homing", intake.getHardware().bucketLiftZeroSwitch.getState());
        }, () -> intake.getHardware().bucketLiftZeroSwitch.getState(), 10000);
        autoHomeTask.addStep(() -> {
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.getHardware().bucketLiftMotor.setTargetPosition(20);
            intake.liftTargetPosition = 20;
            setMotorsToRunConfig(1);
        });
        autoHomeTask.addStep(()->this.setSlideToHomeConfig(2));
        autoHomeTask.addTimedStep(() -> {
            robot.opMode.telemetry.addData("homingH", intake.getHardware().slideZeroSwitch.getState());
        }, () -> intake.getHardware().slideZeroSwitch.getState(), 10000);
        autoHomeTask.addStep(() -> {
            intake.getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.getHardware().horizSliderMotor.setTargetPosition(20);
            intake.slideTargetPosition = 20;
            setMotorsToRunConfig(2);
        });
    }

    private void setSlideToHomeConfig(int i) {
        double power = -.33; //-0.125;
        if (i==1) {
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intake.getHardware().bucketLiftMotor.setPower(power);
        } else if (i==2) {
            intake.getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intake.getHardware().horizSliderMotor.setPower(power);
        }
    }

    private void setMotorsToRunConfig(int i) {
        if (i==1) {
            intake.getHardware().bucketLiftMotor.setPower(IntakeHardware.slideHoldPower);
            intake.getHardware().bucketLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        } else if (i==2) {
            intake.getHardware().horizSliderMotor.setPower(IntakeHardware.slideHoldPower);
            intake.getHardware().horizSliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }



    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autonomousSample = "autonomous sample";
        public final static String autoHome = "auto home";
        public final static String prepareToIntake = "prepare to intake";
        public final static String safe = "safe";
        public final static String transfer = "transfer";
        public final static String prepareToLowDumpIntake = " prepare to low dump for specimen";
        public final static String lowDumpIntake = "low dump intake for specimen";
        public final static String prepareToGetSpecimen = "prepare to get specimen";
        public final static String getSpecimen = "get specimen";
        public final static String hangSpecimen = "hang specimen";
        public final static String prepareToHangSpecimen = "prepare to hang specimen";
        public final static String prepareToDeposit = "prepare to deposit";
        public final static String deposit = "deposit";
        public final static String autoIntake = "auto intake";
        public final static String checkSample = "check sample";
        public final static String ejectBadSample = "eject the sample";
        public final static String prepareToTransfer = "prepare the transfer";
        public final static String prepareToHangRobotTask = "prepare to hang robot";
        public final static String hangRobotTask = "hang robot";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }

//        autoIntakeTask.addStep( ()-> intake.getDistance() < 1.5);
//        autoIntakeTask.addStep( ()-> intake.getSampleType() > 0);
//        autoIntakeTask.addStep( ()-> {
//        if (intake.isSampleGood(intake.lastSample)) prepareToTransferTask.restart();
//        else ejectBadSampleTask.restart();
//    });
//
//    public void constructEjectBadSampleTask() {
//        ejectBadSampleTask.autoStart = false;
//        ejectBadSampleTask.addStep( ()-> intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOut));
//        ejectBadSampleTask.addDelay(1500);
//        ejectBadSampleTask.addStep(autoIntakeTask::restart);
//    }
//
//    public void constructPrepareToTransferTask() {
//        prepareToTransferTask.autoStart = false;
//        prepareToTransferTask.addStep( ()-> intake.getHardware().flipper.setPosition(intake.getSettings().spintakeSafe));
//        prepareToTransferTask.addStep( ()-> intake.getHardware().spinner.setPosition(intake.getSettings().spinnerSlowOut));
//        prepareToTransferTask.addDelay(500);
//        prepareToTransferTask.addStep( ()-> intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff));
//        prepareToTransferTask.addStep(transferTask::restart);
//    }

}
