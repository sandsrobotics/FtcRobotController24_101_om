package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Autonomous (name="LK14273.2 SPECIMEN/HUMAN", group="11")
public class FlipAuto2025LK extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    PositionSolver positionSolver;
    PositionTracker pt;
    Vector3 startPosition;
    Pinpoint odo;
    Intake intake;
    //  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    public static int maxDelay = 3000;
    /**************************/
    public int startDelay;
    private int parkPosition;

    public void initAuto(){
        transformFunc = (v) -> v;
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
        FlipbotSettings.setAuto();
        long start;
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);
        intake = new Intake(robot);

//        Vector3 fieldStartPos = new Vector3(0,0,-90);
        Vector3 fieldStartPos = new Vector3(14 + 3.0/8.0, -62, -90);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        odo = new Pinpoint(pt, true, "pinpoint",
                -56.0, 52.0, 13.26291192f,
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        DecimalFormat df = new DecimalFormat("#0.0");

        robot.init();

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
            // example configuration capability during init
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasTapped)) {
                FlipbotSettings.autonomousDebugMode = !FlipbotSettings.autonomousDebugMode;   //todo: disable this before competition!
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper, ButtonMgr.State.wasTapped)) {
                startDelay += 1000;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.left_bumper, ButtonMgr.State.wasTapped)) {
                startDelay -= 1000;
                if(startDelay < 0) startDelay = 0;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.a, ButtonMgr.State.wasTapped)) {
                parkPosition = 1;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.b, ButtonMgr.State.wasTapped)) {
                parkPosition = 2;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped)) {
                parkPosition = 3;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.y, ButtonMgr.State.wasTapped)) {
                parkPosition = 0;
            }
            if(startDelay > maxDelay) startDelay = maxDelay;

            telemetry.addData("DEBUG?:", FlipbotSettings.autonomousDebugMode ? "***** YES *****" : "No, normal");
            telemetry.addData("PARK POSITION:", parkPosition == 0 ? "Normal mid wall" : parkPosition == 1 ? "Park MID" : parkPosition == 2 ? "Park CORNER" : "Park BOARD");
            telemetry.addData("START DELAY:", startDelay / 1000);
            telemetry.update();
            sleep(50);
        }

        odo.setPosition(fieldStartPos);
        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        // Setting up group container, task queue, and setting positionSolver target
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);
        //positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        // testNewAuto - Successfully Tested!
         testNewAuto(autoTasks);

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            FlipbotSettings.storeRobotPosition(pt.getCurrentPosition());
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("time", System.currentTimeMillis() - start);
            dashboardTelemetry.update();
            telemetry.update();
        }
        robot.stop();
    }

    // testNewAuto - Successfully completed the below steps in 30-seconds.
    //     Hang Pre-loaded Specimen.
    //     Move Sample-1 to Observation Zone.
    //     Move Sample-2 to Observation Zone.
    //     Pickup and Hang Specimen-2.
    //     Pickup and Hang Specimen-3.
    //     Park!
    private void testNewAuto(TimedTask autoTasks) {
        // Positions to travel in SpecAuto
        Vector3 p_1 = new Vector3(14.375, -62, -90);
//        Vector3 p_2 = new Vector3(11.75, -37.75, -90);
        Vector3 p_2 = new Vector3(11.75, -40.25, -90);
        Vector3 p_3 = new Vector3(11.75, -32.75, -90);
        Vector3 p_4 = new Vector3(36, -42, 90);  // Z: -90
        Vector3 p_5 = new Vector3(36, -11.75, 90);
        Vector3 p_6= new Vector3(44.5, -11.75, 90); //Z:180
        Vector3 p_7 = new Vector3(44.5, -52.5, 90); //Z:180
        Vector3 p_pre_8 = new Vector3(44.5, -11.75, 90); // Same as p_6.
        Vector3 p_8 = new Vector3(54.5, -11.75, 90); // Z:180
        Vector3 p_9 = new Vector3(54.5, -50.5, 90); // Z:180
        Vector3 p_post_9 = new Vector3(54.5, -44.5, 90); // Z:180
        Vector3 p_12 = new Vector3(47, -58.5, 90);
        Vector3 p_13 = new Vector3(47, -61.0, 90); // Y:61.5
        Vector3 p_14 = new Vector3(24, -47, 0);
        Vector3 p_15 = new Vector3(8.75, -40.25, -90); // Y:37.75
        Vector3 p_16 = new Vector3(8.75, -32.75 + 1, -90); // Y:32.75
        Vector3 p_17 = new Vector3(5.75, -40.25, -90); // Y:37.75
        Vector3 p_18 = new Vector3(5.75, -32.75 + 1, -90); // Y:32.75

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(p_1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        // Hang Pre-Loaded Specimen.
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
        positionSolver.addMoveToTaskEx(p_2, autoTasks);
        /* LK experiment here */
        specimenHangPositionOnly(autoTasks, p_3,3);

        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
        positionSolver.addMoveToTaskEx(p_2, autoTasks);

        // Move First Sample to ObservationZone.
        positionSolver.addMoveToTaskEx(p_4, autoTasks);
        positionSolver.addMoveToTaskEx(p_5, autoTasks);
        positionSolver.addMoveToTaskEx(p_6, autoTasks);
        positionSolver.addMoveToTaskEx(p_7, autoTasks);

        // Move Second Sample to ObservationZone.
        positionSolver.addMoveToTaskEx(p_pre_8, autoTasks);
        positionSolver.addMoveToTaskEx(p_8, autoTasks);
        positionSolver.addMoveToTaskEx(p_9, autoTasks);

        // Second Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_post_9, p_13, p_14, p_15, p_16);

        // Third Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_17, p_18);

        // Park.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(p_12, autoTasks);
    }

    private void specimenHangPositionOnly (TimedTask autoTasks, Vector3 posHang, double targetDistance) {
        autoTasks.addStep(() -> intake.debugDelay());
        addMove(autoTasks, posHang, 0, false, PositionSolverSettings.defaultTwiceSettingsHang);
        autoTasks.addTimedStep(()-> {
            if (intake.adjustTarget(posHang,targetDistance)) positionSolver.setNewTarget(intake.adjustedDestination, true);
        }, () -> positionSolver.isDone(), 5000);
//        autoTasks.addStep(() -> positionSolver.isDone());
        autoTasks.addStep(() -> intake.debugDelay());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
    }

    private void addMove(TaskEx task, Vector3 target, int timeLimit, boolean wait, PositionSolverSettings psSetting) {
        if (timeLimit < 0) return;
        task.addStep(() -> positionSolver.setSettings(psSetting));
        if (!wait) {
            positionSolver.addMoveToTaskExNoWait(target, task);
        } else if (timeLimit==0) {
            positionSolver.addMoveToTaskEx(target, task);
        } else {
            positionSolver.addMoveToTaskEx(target, task, timeLimit);
        }
    }

    private void specimenPickupAndHang (TimedTask autoTasks, Vector3 pos_two, Vector3 pos_three,
                                        Vector3 pos_four, Vector3 prePosition, Vector3 position) {
        // Specimen Pickup and Hang.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
//        positionSolver.addMoveToTaskEx(pos_three, autoTasks);
        specimenHangPositionOnly(autoTasks, pos_three,1);
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
        positionSolver.addMoveToTaskEx(pos_four, autoTasks);
        positionSolver.addMoveToTaskEx(prePosition, autoTasks);
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
//        positionSolver.addMoveToTaskEx(position, autoTasks);
        specimenHangPositionOnly(autoTasks, position,3);
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
        positionSolver.addMoveToTaskEx(prePosition, autoTasks);
    }


 }