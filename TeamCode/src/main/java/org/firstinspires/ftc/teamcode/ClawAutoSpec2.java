package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

//@Disabled
@Autonomous(name="27050 Specimen Humanside 2", group="27050")
public class ClawAutoSpec2 extends ClawAutoSpec {
    Vector3 observationzonepickup = new Vector3(45.5, -61, 90); // to stop wall hit

    @Override
    public void initAuto() {
        transformFunc = (v) -> v;
        bucketSample = false;
        fieldStartPos = observationzonepickup;
    }

    public void SpecAuto(TimedTask autoTasks) { // from 14273
        // Positions to travel in Auto
        Vector3 p_1 = new Vector3(14.375, -62, -90);
        Vector3 p_2 = new Vector3(11.75, -37.75, -90);
        Vector3 rightbeforespecimenbar = new Vector3(11.75, -39, -90);
        Vector3 p_3 = new Vector3(11.75, -32.75, -90);
        Vector3 p_4 = new Vector3(36, -42, 90);  // Z: -90
        Vector3 p_5 = new Vector3(36, -11.75, 90);
        Vector3 p_6 = new Vector3(44.5, -15, 90); //Z:180
        Vector3 p_7 = new Vector3(44.5, -52.5, 90); //Z:180
        Vector3 p_pre_8 = new Vector3(44.5, -15, 90); // Same as p_6.
        Vector3 p_8 = new Vector3(54.5, -11.75, 90); // Z:180
        Vector3 p_9 = new Vector3(54.5, -52.5, 90); // Z:180
        Vector3 p_new_9 = new Vector3(59.5, -52.5, 90); // Z:180
        Vector3 p_pre_10 = new Vector3(54.5, -11.75, 90); // Same as p_8.
        Vector3 p_10 = new Vector3(61, -11.75, 90); // Z:180
        Vector3 p_11 = new Vector3(61, -52.5, 90); // Z: 180
        Vector3 p_12 = new Vector3(45.5, -58.5, 90);
        Vector3 p_13 = new Vector3(45.5, -61.5, 90); // Y:61.5
        Vector3 p_14 = new Vector3(24, -47, 0);
        Vector3 p_15 = new Vector3(8.75, -37.75 + 1, -90); // Y:37.75
        Vector3 p_16 = new Vector3(8.75, -32.75 + 1, -90); // Y:32.75
        Vector3 p_17 = new Vector3(5.75, -37.75 + 1, -90); // Y:37.75
        Vector3 p_18 = new Vector3(5.75, -32.75 + 1, -90); // Y:32.75
        Vector3 p_19 = new Vector3(2.75, -37.75, -90);
        Vector3 p_20 = new Vector3(2.75, -32.75, -90);
        Vector3 p_21 = new Vector3(-0.25, -37.75, -90);
        Vector3 p_22 = new Vector3(-0.25, -32.75, -90);
        Vector3 p_00 = new Vector3(54, -54, -90);

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> intake.tasks.setMotorsToRunConfig());
        autoTasks.addStep(() -> intake.setHorizontalSlidePosition(-1)); // h-slide in
        autoTasks.addStep(() -> odo.setPosition(p_1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
        positionSolver.addMoveToTaskExNoWait(rightbeforespecimenbar, autoTasks);
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart()); // pickup specimen and raise
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart()); // raise high for specimen hang
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
        autoTasks.addStep(() -> positionSolver.isDone());
        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
        autoTasks.addStep(() -> intake.rangeisDone);
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart()); // clip specimen on bar
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());

        // Move Samples to ObservationZone.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        // First Sample to ObservationZone.
        positionSolver.addMoveToTaskEx(p_4, autoTasks);
        positionSolver.addMoveToTaskEx(p_5, autoTasks);
        positionSolver.addMoveToTaskEx(p_6, autoTasks);
        positionSolver.addMoveToTaskEx(p_7, autoTasks);
        // Second Sample to ObservationZone.
        positionSolver.addMoveToTaskEx(p_pre_8, autoTasks);
        positionSolver.addMoveToTaskEx(p_8, autoTasks);
        positionSolver.addMoveToTaskEx(p_new_9, autoTasks);

        // Second Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_15, p_16);
        // Third Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_17, p_18);
//        // Fourth Specimen PickupAndHang
//        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_19, p_20);
//        // Fifth Specimen PickupAndHang
//        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_21, p_22);
        {
            // Park.
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(p_12, autoTasks);
        }
    }

    private void specimenPickupAndHang(TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three,
                                       Vector3 pos_four, Vector3 prePosition, Vector3 position) {
        // Specimen Pickup and Hang.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks); //p_12
        positionSolver.addMoveToTaskEx(pos_three, autoTasks); //p_13
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(pos_four, autoTasks); //p_14 observationzoneclear
        positionSolver.addMoveToTaskEx(prePosition, autoTasks); // before bar p_15
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
        positionSolver.addMoveToTaskEx(position, autoTasks); // at bar p_16
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(prePosition, autoTasks); //p_15
    }
}

/* *************************************************************************************************************** */
//    @Override
//    public void SpecAuto_rangetest(TimedTask autoTasks) {
//        Vector3 humansidestart = new Vector3(14 + 3.0 / 8.0, -62, -90);
//        Vector3 rightbeforespecimenbar = new Vector3(11.75, -39, -90);
//        Vector3 observationzoneclear = new Vector3(45.5, -56, 90); // to stop wall hit
//
//        autoTasks.addStep(() -> odo.setPosition(observationzonepickup));
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
//        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart()); // pickup specimen and raise
//        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.isDone());
//        positionSolver.addMoveToTaskEx(observationzoneclear, autoTasks);
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
//        positionSolver.addMoveToTaskExNoWait(rightbeforespecimenbar, autoTasks);
//        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart()); // raise high for specimen hang
//        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
//        autoTasks.addStep(() -> positionSolver.isDone());
//        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
//        autoTasks.addStep(() -> intake.rangeisDone);
//        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart()); // clip specimen on bar
//        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
//        autoTasks.addStep(() -> intake.getHardware().backLight.setPosition(.7));
//    }