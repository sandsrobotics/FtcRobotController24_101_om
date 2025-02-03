package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import java.util.Objects;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

//@Disabled
@Autonomous(name="27050 Specimen Humanside 2", group="27050")
public class ClawAutoSpec2 extends ClawAutoSpec {
    @Override
    public void initAuto() {
        transformFunc = (v) -> v;
        bucketSample = false;
    }

@Override
public void SpecAuto(TimedTask autoTasks) {
        Vector3 humansidestart = new Vector3(14 + 3.0 / 8.0, -62, -90);
        Vector3 rightbeforespecimenbar = new Vector3(11.75, -39, -90);
        Vector3 observationzonepickup = new Vector3(47, -61.5, 90); // to stop wall hit
        Vector3 observationzoneclear = new Vector3(47, -52, 90); // to stop wall hit


        autoTasks.addStep(() -> odo.setPosition(observationzonepickup));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
        positionSolver.addMoveToTaskEx(observationzoneclear, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoSpecimenPickupTask.restart()); // pickup specimen and raise
        autoTasks.addStep(() -> intake.tasks.autoSpecimenPickupTask.isDone());
        positionSolver.addMoveToTaskExNoWait(rightbeforespecimenbar, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoSpecimenSetTask.restart()); // raise high for specimen hang
        autoTasks.addStep(() -> intake.tasks.autoSpecimenSetTask.isDone());
        autoTasks.addStep(() -> positionSolver.isDone());
        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
        autoTasks.addStep(() -> intake.rangeisDone);
        autoTasks.addStep(() -> intake.tasks.autoSpecimenHangTask.restart()); // clip specimen on bar
        autoTasks.addStep(() -> intake.tasks.autoSpecimenHangTask.isDone());
        autoTasks.addStep(() -> intake.getHardware().backLight.setPosition(.7));
    }

    public void SpecAuto_old(TimedTask autoTasks) {
        Vector3 humansidestart = new Vector3(14 + 3.0/8.0, -62, -90);
        Vector3 rightbeforespecimenbar = new Vector3(11.75, -39, -90);
        Vector3 rightbeforespecimenbar2 = new Vector3(8.75, -39, -90);
        Vector3 rightbeforespecimenbar3 = new Vector3(5.75, -39, -90);
        Vector3 specimenbar = new Vector3(11.75, -32.75, -90);
        Vector3 specimenbar2 = new Vector3(8.75, -32.75, -90);
        Vector3 specimenbar3 = new Vector3(5.75, -32.75, -90);
        Vector3 afterfirstredbar = new Vector3(32, -42, 180);
        Vector3 afterfirstredbar2 = new Vector3(35, -42, 180);
        Vector3 rightbeforesample = new Vector3(36.5, -11.75, 180);
        Vector3 atfirstsample = new Vector3(43.5, -11.75, 90);
        Vector3 observationzone1 = new Vector3(43.5, -52, 90);
        Vector3 beforesecondsample = new Vector3(43.5, -11.75, 90);
        Vector3 atsecondsample = new Vector3(53.5, -11.75, 90);
        Vector3 observationzone2 = new Vector3(53.5, -52, 90);
        Vector3 observationzoneprepickup = new Vector3(47, -58.5, 90);
        Vector3 observationzoneprepickup2 = new Vector3(42, -40.0, 90);
        Vector3 midwayspecimen2hang = new Vector3(24, -47, 0);
        Vector3 observationzonepickup = new Vector3(47, -61.5, 90); // to stop wall hit
        Vector3 beforethirdsample = new Vector3(44.5, -11.75, 180);
        Vector3 atthirdsample = new Vector3(61, -11.75, 180);
        Vector3 observationzone3 = new Vector3(61, -52.5, 180);
        Vector3 beforespecimen2 = new Vector3(46, -52.5, 180);
        Vector3 rotationbeforespecimen2 = new Vector3(46, -52.5, 90);
        Vector3 atspecimen2 = new Vector3(46, -61.5, 90);
        Vector3 specimen2hang = new Vector3(8.75, -32.75, -90);
        Vector3 backmidwayspecimen2spot = new Vector3(23.5, -47, 0); // mid long delay
        Vector3 atspecimen3 = new Vector3(46, -61.5, 90);
        Vector3 midwayspecimen3hang = new Vector3(23.5, -47, 0);
        Vector3 specimen3hang = new Vector3(5.75, -32.75, -90);
        Vector3 parkingposition = new Vector3(54, -54, 0);

        autoTasks.addStep(()-> intake.stopAllIntakeTasks());
        autoTasks.addStep(()-> odo.setPosition(humansidestart));
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowSettings));
        // close pincer on initial specimen
        autoTasks.addStep(() -> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        positionSolver.addMoveToTaskExNoWait(rightbeforespecimenbar, autoTasks);
        autoTasks.addStep(()-> intake.rangeEnabled=true);
        autoTasks.addStep(()-> intake.rangeisDone);
        autoTasks.addStep(() -> intake.tasks.autoSpecimenSetTask.restart()); // prepare for specimen hang
        autoTasks.addStep(() -> intake.tasks.autoSpecimenSetTask.isDone()); // prepare for specimen hang
        positionSolver.addMoveToTaskEx(specimenbar, autoTasks);
        // clip specimen on bar
        autoTasks.addStep(() -> intake.tasks.autoSpecimenPickupTask.restart());
        autoTasks.addDelay(200);
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        autoTasks.addDelay(200);
       positionSolver.addMoveToTaskEx(afterfirstredbar, autoTasks);
      positionSolver.addMoveToTaskEx(rightbeforesample, autoTasks);
        positionSolver.addMoveToTaskEx(atfirstsample, autoTasks);
       positionSolver.addMoveToTaskEx(observationzone1, autoTasks);
            positionSolver.addMoveToTaskEx(afterfirstredbar2, autoTasks);
            positionSolver.addMoveToTaskEx(beforesecondsample, autoTasks);
            positionSolver.addMoveToTaskEx(atsecondsample, autoTasks);
            positionSolver.addMoveToTaskEx(observationzone2, autoTasks);
            positionSolver.addMoveToTaskEx(observationzoneprepickup2, autoTasks);
           autoTasks.addDelay(200);   positionSolver.addMoveToTaskEx(observationzoneprepickup, autoTasks);
//        {
//            // Second Specimen Hang
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
//            positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
//            autoTasks.addDelay(200);
//            // close pincer on initial specimen
//            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenPickup()); // grab specimen
//            autoTasks.addDelay(250);
//            positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
//            autoTasks.addDelay(250);
//            positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
//            autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
//            autoTasks.addDelay(200);
//            positionSolver.addMoveToTaskEx(specimenbar2, autoTasks);
//            autoTasks.addDelay(200);
//            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
//            autoTasks.addDelay(200);
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
//            positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
//        }
//        {
//            //Moving Third Sample.

//
//            // Third Specimen Hang.
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
//            positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
//            autoTasks.addDelay(200);
//            autoTasks.addStep(() -> intake.tasks.autoSpecimenPickupTask.restart());
////            autoTasks.addDelay(250);
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
//            positionSolver.addMoveToTaskEx(midwayspecimen3hang, autoTasks);
////            autoTasks.addDelay(250);
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
//            positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
//            autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
//            autoTasks.addDelay(200);
//            positionSolver.addMoveToTaskEx(specimenbar3, autoTasks);
//            autoTasks.addDelay(200);
//            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
//            autoTasks.addDelay(200);
//            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
//            positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
//        }
//        {
//            // Park
//            positionSolver.addMoveToTaskEx(parkingposition, autoTasks);
//        }
    }
}