package org.firstinspires.ftc.teamcode.parts.intake2;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake2Tasks {
    private final Group movementTask;
    private final TimedTask autoHomeTask;
    private Intake2 intake;
    private Robot robot;

    public Intake2Tasks(Intake2 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
    }

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

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autoHome = "auto home";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
