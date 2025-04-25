package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import java.util.Objects;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

//@Disabled
@Autonomous(name="27050 Bucket Specimen", group="27050")
public class ClawAutoBucket extends ClawAutoSpec {
    Vector3 bucketsidestart = new Vector3(-14 - 3.0 / 8.0, -62, -90);
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
        bucketSample = false;
        fieldStartPos =bucketsidestart;
    }

    @Override
    public void BucketAuto(TimedTask autoTasks) {
        Vector3 bucketsamplestart = new Vector3(-37.5, -62, 90);
        Vector3 beforespecimenhang = new Vector3(-10, -39, -90);
        Vector3 specimenhang = new Vector3(-10, -32.75, -90); //specimen must be lifted before hang
        firstsample = new Vector3(-47.3, -38.5, 90); // -47.8
        Vector3 Highbasketscore = new Vector3(-54.5, -53.5, 45);
        Vector3 Highbasketscorethird = new Vector3(-53.5, -52.5, 45);
        secondsample = new Vector3(-57.7, -38.5, 90); // -58.2
        thirdsample = new Vector3(-56, -25.25, 180);
        Vector3 park = new Vector3(-47, -11, 0);
        Vector3 park2 = new Vector3(-25, -11,0); //x change x was 27 was 23.5
        // 23.5 x -1.5, 23.5 x -1.5

        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
        autoTasks.addStep(() -> intake.tasks.setMotorsToRunConfig());
        autoTasks.addStep(()-> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmSafe)); // just outsid the 18"
        autoTasks.addStep(() -> intake.setHorizontalSlidePosition(-1)); // h-slide in

        if (!bucketSample) { //Spec side of bucket
            autoTasks.addStep(() -> odo.setPosition(bucketsidestart));
            autoTasks.addStep(() -> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
            positionSolver.addMoveToTaskExNoWait(beforespecimenhang, autoTasks);
            autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart()); // prepare for specimen hang
            autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
            positionSolver.addMoveToTaskEx(specimenhang, autoTasks);
            //        autoTasks.addDelay(200);
            autoTasks.addStep( () -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(beforespecimenhang, autoTasks);
        } else {
            autoTasks.addStep(() -> odo.setPosition(bucketsamplestart));
            positionSolver.addMoveToTaskExNoWait(Highbasketscore, autoTasks);
            autoTasks.addStep(() -> intake.tasks.autoBucketLiftTask.restart());
            autoTasks.addStep(() -> intake.tasks.autoBucketLiftTask.isDone());

            autoTasks.addStep(() -> intake.tasks.autoBucketDropperTask.restart());
            autoTasks.addStep(() -> intake.tasks.autoBucketDropperTask.isDone());
        }
        grabAndDepositSample(autoTasks, firstsample, Highbasketscore);
        grabAndDepositSample(autoTasks, secondsample, Highbasketscore);
        grabAndDepositSample(autoTasks, thirdsample, Highbasketscorethird);

        autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().minLiftPosition, 1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        autoTasks.addStep(() -> intake.getHardware().parkServo.setPosition(intake.getSettings().parkServoPositionParked));
        positionSolver.addMoveToTaskEx(park, autoTasks);
        autoTasks.addStep(() -> intake.getHardware().tiltServo.setPosition(intake.getSettings().intakeArmAtBucket));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(park2, autoTasks);
        autoTasks.addStep(() -> intake.getHardware().parkServo.setPosition(intake.getSettings().parkServoPositionParked -.02));
    }

    private void grabAndDepositSample (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two) {
        if(Objects.equals(pos_one,firstsample) && !bucketSample) {
            positionSolver.addMoveToTaskEx(pos_one, autoTasks);
        } else {
            positionSolver.addMoveToTaskExNoWait(pos_one, autoTasks);
        }
        // added dropperDockSafe for safety might be too slow
        autoTasks.addStep(()-> intake.getHardware().dropperServo.setPosition(intake.getSettings().dropperDockSafe));
        autoTasks.addStep(()-> intake.getHardware().dropperServo.isDone());
        autoTasks.addStep(()-> intake.getHardware().dropperServo.disable());
        autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().minLiftPosition, 1));
        autoTasks.addStep(intake::isLiftInTolerance);

        if (Objects.equals(pos_one, thirdsample)) {
            autoTasks.addStep(() -> intake.tasks.autoSamplePickupTaskHack.restart());
            autoTasks.addStep(() -> intake.tasks.autoSamplePickupTaskHack.isDone());
        } else {
            autoTasks.addStep(() -> intake.tasks.autoSamplePickupTask.restart());
            autoTasks.addStep(() -> intake.tasks.autoSamplePickupTask.isDone());
        }
        positionSolver.addMoveToTaskExNoWait(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoBucketLiftTask.restart());
        autoTasks.addStep( () -> intake.tasks.autoBucketLiftTask.isDone());

        autoTasks.addStep(() -> intake.tasks.autoBucketDropperTask.restart());
        autoTasks.addStep(() -> intake.tasks.autoBucketDropperTask.isDone());
    }
}
