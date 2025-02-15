package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake2.Intake2;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import java.text.DecimalFormat;
import java.util.Objects;
import java.util.function.Function;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.suppliers.EdgeSupplier;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;
import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Autonomous(name="27050 Human Specimen", group="27050")
public class ClawAutoSpec extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    public boolean bucketSide = false;
    public boolean bucketSample = false;
    PositionSolver positionSolver;
    PositionTracker pt;
    Vector3 startPosition;
    Vector3 firstsample;
    Vector3 secondsample;
    Vector3 thirdsample;
    Pinpoint odo;
    Intake2 intake;
    //  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    public static int maxDelay = 3000;
    /**************************/
    public int startDelay;
    private int parkPosition;
    Vector3 fieldStartPos;

    public void initAuto(){
        transformFunc = (v) -> v;
        fieldStartPos = new Vector3(14.375, -62, -90);
        //Vector3 humansidestart = new Vector3(14 + 3.0/8.0, -62, -90);
    }

    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    private Vector3 tileToInchAutoNoZ(Vector3 tiles){ return Constants.tileToInch(transformFunc.apply(tiles)).withZ(tiles.Z); }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    @Override
    public void runOpMode() {
        long start;
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);
        intake = new Intake2(robot, "Autonomous");

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2, 2, 2), fieldStartPos);
        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
        odo = new Pinpoint(pt);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        DecimalFormat df = new DecimalFormat("#0.0");

        robot.init();

        while (!isStarted()) {
//            robot.buttonMgr.runLoop();
////            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper))
//            if (new EdgeSupplier(() -> robot.opMode.gamepad1.right_bumper).isRisingEdge()) {
//                startDelay += 1000;
//            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.left_bumper).isRisingEdge()) {
//                startDelay -= 1000;
//                if (startDelay < 0) startDelay = 0;
//            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.a).isRisingEdge()) {
//                parkPosition = 1;
//            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.b).isRisingEdge()) {
//                parkPosition = 2;
//            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.x).isRisingEdge()) {
//                parkPosition = 3;
//            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.y).isRisingEdge()) {
//                parkPosition = 0;
//            }
//
//            if(startDelay > maxDelay) startDelay = maxDelay;
//
//            telemetry.addData("PARK POSITION:", parkPosition == 0 ? "Park based off tags" : parkPosition == 1 ? "Park MID" : parkPosition == 2 ? "Park CORNER" : "Park BOARD");
//            telemetry.addData("START DELAY:", startDelay / 1000);
//            dashboardTelemetry.update();
//            telemetry.update();
            sleep(50);
        }
        odo.setPosition(fieldStartPos);
        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        // Setting up group container, task queue, and setting positionSolver target
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);

        if (bucketSide)
            BucketAuto(autoTasks);
        else {
            SpecAuto(autoTasks);
        }

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("time", System.currentTimeMillis() - start);
            dashboardTelemetry.update();
            telemetry.update();
        }
        robot.stop();
    }
    public void BucketAuto(TimedTask autoTasks) {} // this method is in Claw Auto Bucket

    public void SpecAuto(TimedTask autoTasks) { // from 14273
        // Positions to travel in Auto
        Vector3 p_1 = new Vector3(14.375, -62, -90);
        Vector3 p_2 = new Vector3(11.75, -37.75, -90);
        Vector3 rightbeforespecimenbar = new Vector3(11.75, -39, -90);
        Vector3 p_3 = new Vector3(11.75, -32.75, -90);
        Vector3 p_4 = new Vector3(36, -43, 90);  // Z: -90
        Vector3 p_5 = new Vector3(36, -15, 90);
        Vector3 p_6 = new Vector3(42.5, -15, 90); //Z:180
        Vector3 p_7 = new Vector3(42.5, -50.5, 90); //Z:180
        Vector3 p_pre_8 = new Vector3(42.5, -15, 90); // Same as p_6.
        Vector3 p_8 = new Vector3(52.5, -15, 90); // Z:180
        Vector3 p_9 = new Vector3(52.5, -49.5, 90); // Z:180
        Vector3 p_post_9 = new Vector3(52.5, -44.5, 90); // Z:180
        Vector3 p_12 = new Vector3(33.5, -56.5, 90); // Y:-58.5 moved pickup 10" to left
        Vector3 p_13 = new Vector3(33.5, -61.5, 90); // Y:61.5 moved pickup 10" to left
        Vector3 p_14 = new Vector3(24, -47, 0);
        Vector3 p_15 = new Vector3(8.75, -39, -90); // Y:37.75
        Vector3 p_16 = new Vector3(8.75, -32.75 + 1, -90); // Y:32.75
        Vector3 p_17 = new Vector3(5.75, -39, -90); // Y:37.75
        Vector3 p_18 = new Vector3(5.75, -32.75 + 1, -90); // Y:32.75
        Vector3 p_19 = new Vector3(2.75, -37.75, -90);
        Vector3 p_20 = new Vector3(2.75, -32.75, -90);
        Vector3 p_00 = new Vector3(45.5, -56.5, -45); // Y:-58.5

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> intake.tasks.setMotorsToRunConfig());
        autoTasks.addStep(() -> intake.setHorizontalSlidePosition(-1)); // h-slide in
        autoTasks.addStep(() -> odo.setPosition(p_1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
        positionSolver.addMoveToTaskExNoWait(rightbeforespecimenbar, autoTasks);
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart()); // raise high for specimen hang
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
        autoTasks.addStep(() -> intake.rangeisDone);
        autoTasks.addStep(() -> intake.tasks.autoSpecimenHang1.restart()); // clip specimen on bar
        autoTasks.addStep(() -> intake.tasks.autoSpecimenHang1.isDone());

        // Move Samples to ObservationZone.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        // First Sample to ObservationZone.
        positionSolver.addMoveToTaskEx(p_4, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoSpecimenHang2.restart());
        positionSolver.addMoveToTaskEx(p_5, autoTasks);
        positionSolver.addMoveToTaskEx(p_6, autoTasks);
        positionSolver.addMoveToTaskEx(p_7, autoTasks);
        // Second Sample to ObservationZone.
        positionSolver.addMoveToTaskEx(p_pre_8, autoTasks);
        positionSolver.addMoveToTaskEx(p_8, autoTasks);
        positionSolver.addMoveToTaskEx(p_9, autoTasks);

        // Second Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_15, p_16);
        // Third Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_17, p_18);
        // Fourth Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_7, p_12, p_13, p_14, p_19, p_20);
        // Park.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(p_00, autoTasks); // angle straight to park
    }

    private void specimenPickupAndHang(TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three,
                                       Vector3 pos_four, Vector3 prePosition, Vector3 position) {
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks); //p_12  // before observation zone
        positionSolver.addMoveToTaskEx(pos_three, autoTasks); //p_13 // at observation zone
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
        positionSolver.addMoveToTaskEx(prePosition, autoTasks); // before bar p_15, p_17, p_19
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
        autoTasks.addStep(() -> intake.rangeisDone);
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
//        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
        autoTasks.addDelay(70);
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
    }
}
