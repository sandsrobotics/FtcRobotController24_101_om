package org.firstinspires.ftc.teamcode;

import static om.self.ezftc.utils.Constants.tileSide;

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
import om.self.task.other.TimedTask;

@Config
@Autonomous (name="14273.2 SPECIMEN/HUMAN", group="14273")
public class FlipAuto2025 extends LinearOpMode{
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
    public long startTime;

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

        // Here is where we schedule the tasks for the autonomous run (testAuto function below run loop)
        // testAuto(autoTasks);

        // testNewAuto - Successfully Tested!
         testNewAuto(autoTasks);

        //testNewAutoWithIntake needs Testing!
        // testNewAutoWithIntake(autoTasks);
        startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
//            start = System.currentTimeMillis();
            robot.run();
            FlipbotSettings.storeRobotPosition(pt.getCurrentPosition());
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("time", System.currentTimeMillis() - startTime);
            dashboardTelemetry.update();
            telemetry.update();
        }
        robot.stop();
    }

    private void testAuto(TimedTask autoTasks) {
        Vector3 humansidestart = new Vector3(14 + 3.0/8.0, -62, -90);
        Vector3 bucketsidestart = new Vector3(-14.373, -62, 90);
        Vector3 rightbeforespecimenbar = new Vector3(11.75, -37.75, -90);
        Vector3 rightbeforespecimenbar2 = new Vector3(8.75, -37.75, -90);

        Vector3 rightbeforespecimenbar3 = new Vector3(5.75, -37.75, -90);
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

        Vector3 observationzonepickup = new Vector3(47, -62, 90);
        Vector3 beforethirdsample = new Vector3(44.5, -11.75, 180);
        Vector3 atthirdsample = new Vector3(61, -11.75, 180);
        Vector3 observationzone3 = new Vector3(61, -52.5, 180);
        Vector3 beforespecimen2 = new Vector3(46, -52.5, 180);
        Vector3 rotationbeforespecimen2 = new Vector3(46, -52.5, 90);
        Vector3 atspecimen2 = new Vector3(46, -61.5, 90);
        Vector3 specimen2hang = new Vector3(8.75, -32.75, -90);
        Vector3 backmidwayspecimen2spot = new Vector3(23.5, -47, 0);
        Vector3 atspecimen3 = new Vector3(46, -61.5, 90);
        Vector3 midwayspecimen3hang = new Vector3(23.5, -47, 0);
        Vector3 specimen3hang = new Vector3(5.75, -32.75, -90);
        Vector3 parkingposition = new Vector3(54, -54, 0);

        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(humansidestart));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
        // close pincer on initial specimen
        //autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());  // lk this raises the lift a little too, not needed here
        autoTasks.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchClosed));
        autoTasks.addStep(() -> intake.getHardware().pinch.isDone());
          autoTasks.addStep(() -> intake.debugDelay());
          //autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        // raise for specimen hang
          autoTasks.addStep(() -> intake.debugDelay());
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
        autoTasks.addDelay(200);
        positionSolver.addMoveToTaskEx(specimenbar, autoTasks);
        autoTasks.addDelay(200);
        // clip specimen on bar
          autoTasks.addStep(() -> intake.debugDelay());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addDelay(200);
          autoTasks.addStep(() -> intake.debugDelay());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        autoTasks.addDelay(200);
          autoTasks.addStep(() -> intake.debugDelay());
        positionSolver.addMoveToTaskEx(afterfirstredbar, autoTasks);
          autoTasks.addStep(() -> intake.debugDelay());
        positionSolver.addMoveToTaskEx(rightbeforesample, autoTasks);
          autoTasks.addStep(() -> intake.debugDelay());
        positionSolver.addMoveToTaskEx(atfirstsample, autoTasks);
          autoTasks.addStep(() -> intake.debugDelay());
        positionSolver.addMoveToTaskEx(observationzone1, autoTasks);
          autoTasks.addStep(() -> intake.debugDelay());
        positionSolver.addMoveToTaskEx(observationzoneprepickup, autoTasks);
        {
            // Second Specimen Hang
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
            positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
            autoTasks.addDelay(200);
            // close pincer on initial specimen
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());  // should wait for this complete, or figure exact delay
            autoTasks.addDelay(250);
              autoTasks.addStep(() -> intake.debugDelay());
              //autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));  // maybe too wobbly?
            positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
            autoTasks.addDelay(250);
              autoTasks.addStep(() -> intake.debugDelay());
              //autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));  // here might work better?
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(specimenbar2, autoTasks);
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart()); // should wait for this complete, or figure exact delay
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
        }
        {
            //Moving Third Sample.
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(afterfirstredbar2, autoTasks);
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(beforesecondsample, autoTasks);
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(atsecondsample, autoTasks);
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(observationzone2, autoTasks);
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(observationzoneprepickup2, autoTasks);
            autoTasks.addDelay(200);


            // Third Specimen Hang.
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
            positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());  // should wait for this complete, or figure exact delay
            autoTasks.addDelay(250);
              autoTasks.addStep(() -> intake.debugDelay());
              //autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));  // maybe too wobbly?
            positionSolver.addMoveToTaskEx(midwayspecimen3hang, autoTasks);
            autoTasks.addDelay(250);
              autoTasks.addStep(() -> intake.debugDelay());
              //autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));  // here might work better?
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
            autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(specimenbar3, autoTasks);
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart()); // should wait for this complete, or figure exact delay
            autoTasks.addDelay(200);
              autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
        }
        {
            // Park.
              autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(parkingposition, autoTasks);
        }
}

    // testNewAuto - Four-Specimen Hang and Park.
    //     Hang Pre-loaded Specimen.
    //     Move Sample-1 to Observation Zone.
    //     Move Sample-2 to Observation Zone.
    //     Pickup and Hang Specimen-2.
    //     Pickup and Hang Specimen-3.
    //     Pickup and Hang Specimen-4.
    //     Park!
    private void testNewAuto(TimedTask autoTasks) {
        // Positions to travel in SpecAuto
        Vector3 p_1 = new Vector3(14.375, -62, -90);
        Vector3 p_2 = new Vector3(11.75, -40.25 + 5, -90); // Y:-37.75
        Vector3 p_3 = new Vector3(11.75, -32.75 + 1, -90);
        Vector3 p_4 = new Vector3(36, -43, 90);  // Z: -90
        Vector3 p_5 = new Vector3(36, -15, 90); // Y:-11.75
        Vector3 p_6 = new Vector3(44.5, -15, 90); //Y:-11.75, Z:180
        Vector3 p_7 = new Vector3(44.5, -50.5, 90); //Y:-52.5, Z:180
        Vector3 p_pre_8 = new Vector3(44.5, -15, 90); // Same as p_6.
        Vector3 p_8 = new Vector3(52.5, -15, 90); // X:54.5, Y:-11.75, Z:180
        Vector3 p_9 = new Vector3(52.5, -49.5, 90); // X:54.5, Y:-52.5, Z:180
        Vector3 post_9 = new Vector3(52.5, -44.5, 90); // X:54.5
        Vector3 p_12 = new Vector3(33.5, -56.5, 90); // X:45.5
        Vector3 p_13 = new Vector3(33.5, -61.5, 90); // X:45.5, Y:61.5
        Vector3 p_14 = new Vector3(24, -47, 0);
        Vector3 p_15 = new Vector3(8.75, -40.25 + 5, -90); // Y:37.75
        Vector3 p_16 = new Vector3(8.75, -32.75 + 1, -90); // Y:32.75
        Vector3 p_17 = new Vector3(5.75 - 3, -40.25 + 5 , -90); // Y:37.75
        Vector3 p_18 = new Vector3(5.75 - 3, -32.75 + 1, -90); // Y:32.75
        Vector3 p_19 = new Vector3(2.75 - 6, -40.25 + 5 , -90); // Y:37.75
        Vector3 p_20 = new Vector3(2.75 - 6, -32.75 + 1, -90); // Y:32.75
        Vector3 p_00 = new Vector3(55.5, -56.5, 0); // Y:32.75

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(p_1));
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
        // Hang Pre-Loaded Specimen.
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
        positionSolver.addMoveToTaskEx(p_2, autoTasks);
//        specimenPositionWithRange(autoTasks, p_3, 3);
//        positionSolver.addMoveToTaskEx(p_3, autoTasks);
        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
        autoTasks.addStep(() -> intake.rangeIsDone);
//        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
//        positionSolver.addMoveToTaskEx(p_2, autoTasks);

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));

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
        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_15, p_16);

        //cw
        positionSolver.addMoveToTaskExNoWait(p_12.withZ(0), autoTasks);
        autoTasks.addDelay(150);

        // Third Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_17, p_18);

        //cw
        positionSolver.addMoveToTaskExNoWait(p_12.withZ(0), autoTasks);
        autoTasks.addDelay(150);

        // Fourth Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_19, p_20);

        // Park.
        if ((double) (System.currentTimeMillis() - startTime) /1000 < 3.5) {
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(p_19, autoTasks);  //p_00
        } else {
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(p_00, autoTasks);  //p_00
        }
    }

    private void specimenPickupAndHang (TimedTask autoTasks, Vector3 pos_two, Vector3 pos_three,
                                        Vector3 pos_four, Vector3 prePosition, Vector3 position) {
        // Specimen Pickup and Hang.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
//        specimenPositionWithRange(autoTasks, pos_three, 1);
        positionSolver.addMoveToTaskEx(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.getSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
//        positionSolver.addMoveToTaskEx(pos_four, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceNoAlwaysRunSettings));
        ////CCW
        positionSolver.addMoveToTaskExNoWait(prePosition.withZ(0), autoTasks);
        autoTasks.addDelay(150);
        positionSolver.addMoveToTaskEx(prePosition, autoTasks);
//        specimenPositionWithRange(autoTasks, position, 3);
//        positionSolver.addMoveToTaskEx(position, autoTasks);
        autoTasks.addStep(() -> intake.rangeEnabled = true); // range to bar
        autoTasks.addStep(() -> intake.rangeIsDone);
//        autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.isDone());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.isDone());
//        positionSolver.addMoveToTaskEx(prePosition, autoTasks);
    }

    private void specimenPositionWithRange (TimedTask autoTasks, Vector3 targetPosition, double targetDistance) {
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskExNoWait(targetPosition, autoTasks);
        autoTasks.addTimedStep(() -> {},() -> intake.adjustTarget(targetPosition,targetDistance) || positionSolver.isDone(),2000 );
        autoTasks.addStep(() -> {
            if (intake.adjustedDestination!=null) positionSolver.setNewTarget(intake.adjustedDestination, true);
        });
        autoTasks.addStep(() -> positionSolver.isDone());
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
    }

    private void testNewAutoWithIntake(TimedTask autoTasks) {

        // Positions to travel in Auto
        Vector3 p_1 = new Vector3(14.375, -62, -90);
        Vector3 p_2 = new Vector3(11.75, -37.75, -90);
        Vector3 p_3 = new Vector3(11.75, -32.75, -90);
        Vector3 p_4 = new Vector3(36, -42, -90);

        Vector3 p_4_1 = new Vector3(32.5, -40.5, 40);
        Vector3 p_4_2 = new Vector3(32.5, -40.5, -45);
        Vector3 p_4_3 = new Vector3(41.5, -40.5, 40);
        Vector3 p_4_4 = new Vector3(41.5, -40.5, -45);

        Vector3 p_5 = new Vector3(36, -11.75, -90);
        Vector3 p_6= new Vector3(44.5, -11.75, 180);
        Vector3 p_7 = new Vector3(44.5, -52.5, 0); // Z:180
        Vector3 p_pre_8 = new Vector3(44.5, -11.75, 180); // Same as p_6.
        Vector3 p_8 = new Vector3(54.5, -11.75, 180);
        Vector3 p_9 = new Vector3(54.5, -52.5, 180);
        Vector3 p_pre_10 = new Vector3(44.5, -11.75, 180); // Same as p_6.
        Vector3 p_10 = new Vector3(61, -11.75, 180);
        Vector3 p_11 = new Vector3(61, -52.5, 180);
        Vector3 p_12 = new Vector3(47, -58.5, 90);
        Vector3 p_13 = new Vector3(47, -61.5, 90);
        Vector3 p_14 = new Vector3(24, -47, 0);
        Vector3 p_15 = new Vector3(8.75, -37.75, -90);
        Vector3 p_16 = new Vector3(8.75, -32.75, -90);
        Vector3 p_17 = new Vector3(5.75, -37.75, -90);
        Vector3 p_18 = new Vector3(5.75, -32.75, -90);
        Vector3 p_19 = new Vector3(2.75, -37.75, -90);
        Vector3 p_20 = new Vector3(2.75, -32.75, -90);
        Vector3 p_21 = new Vector3(-0.25, -37.75, -90);
        Vector3 p_22 = new Vector3(-0.25, -32.75, -90);
        Vector3 p_00 = new Vector3(54, -54, -90);

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(p_1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        {
            // Pre-Loaded Specimen.
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.prepareToHangSpecimenTask.restart());
            positionSolver.addMoveToTaskEx(p_2, autoTasks);
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
            positionSolver.addMoveToTaskEx(p_3, autoTasks);
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.hangSpecimenTask.restart());
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(p_2, autoTasks);
        }

        {
            // Move Samples to ObservationZone.
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
            autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor));

            // First Sample to ObservationZone.
            autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(p_4_1, autoTasks);
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakePickupTask.restart());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakePickupTask.isDone());

            autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(p_4_2, autoTasks);
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakeDepositTask.restart());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakeDepositTask.isDone());

            // Move Second Sample to ObservationZone.
            autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(p_4_3, autoTasks);
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakePickupTask.restart());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakePickupTask.isDone());

            autoTasks.addStep(() -> intake.debugDelay());
            positionSolver.addMoveToTaskEx(p_4_4, autoTasks);
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakeDepositTask.restart());
            autoTasks.addStep(() -> intake.tasks.specAutoIntakeDepositTask.isDone());

            // TODO: Move Third Sample to ObservationZone.

            // Park Flipper.
            autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperParked));
        }

        // Second Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_15, p_16);

        // Third Specimen PickupAndHang
        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_17, p_18);

        // Fourth Specimen PickupAndHang
//        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_19, p_20);

        // Fifth Specimen PickupAndHang
//        specimenPickupAndHang(autoTasks, p_12, p_13, p_14, p_21, p_22);

        {
            // Park.
            autoTasks.addStep(() -> intake.debugDelay());
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(p_12, autoTasks);
        }
    }
 }