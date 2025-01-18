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
import java.util.function.Function;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.suppliers.EdgeSupplier;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;
import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Autonomous(name="1A Claw Spec Auto", group="Test")
public class ClawAuto2025 extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    public boolean bucketSide = false;
    PositionSolver positionSolver;
    PositionTracker pt;
    Vector3 startPosition;
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

//        Vector3 fieldStartPos = new Vector3(0,0,-90);
        Vector3 fieldStartPos = new Vector3(14 + 3.0 / 8.0, -62, -90);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2, 2, 2), fieldStartPos);
        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
        odo = new Pinpoint(pt);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        DecimalFormat df = new DecimalFormat("#0.0");

        robot.init();

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
//            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper))
            if (new EdgeSupplier(() -> robot.opMode.gamepad1.right_bumper).isRisingEdge()) {
                startDelay += 1000;
            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.left_bumper).isRisingEdge()) {
                startDelay -= 1000;
                if (startDelay < 0) startDelay = 0;
            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.a).isRisingEdge()) {
                parkPosition = 1;
            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.b).isRisingEdge()) {
                parkPosition = 2;
            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.x).isRisingEdge()) {
                parkPosition = 3;
            } else if (new EdgeSupplier(() -> robot.opMode.gamepad1.y).isRisingEdge()) {
                parkPosition = 0;
            }

            if(startDelay > maxDelay) startDelay = maxDelay;

            dashboardTelemetry.addData("position", "blah");
            telemetry.addData("PARK POSITION:", parkPosition == 0 ? "Park based off tags" : parkPosition == 1 ? "Park MID" : parkPosition == 2 ? "Park CORNER" : "Park BOARD");
            telemetry.addData("START DELAY:", startDelay / 1000);
            dashboardTelemetry.update();
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
        if (bucketSide)
            BucketAuto(autoTasks);
        else {
            SpecAuto(autoTasks);
//            testAuto2(autoTasks);
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

    private void SpecAuto_old(TimedTask autoTasks) {
        Vector3 humansidestart = new Vector3(14 + 3.0 / 8.0, -62, -90);
        Vector3 rightbeforespecimenbar = new Vector3(11.75, -37.75, -90);
        Vector3 specimenbar = new Vector3(11.75, -32.75, -90);
        Vector3 afterfirstredbar = new Vector3(36, -42, -90);
        Vector3 rightbeforesample = new Vector3(36, -11.75, -90);
        Vector3 atfirstsample = new Vector3(44.5, -11.75, 180);
        Vector3 observationzone1 = new Vector3(44.5, -52.5, 180);
        Vector3 beforesecondsample = new Vector3(44.5, -11.75, 180);
        Vector3 atsecondsample = new Vector3(54.5, -11.75, 180);
        Vector3 observationzone2 = new Vector3(54.5, -52.5, 180);
        Vector3 beforethirdsample = new Vector3(44.5, -11.75, 180);
        Vector3 atthirdsample = new Vector3(61, -11.75, 180);
        Vector3 observationzone3 = new Vector3(61, -52.5, 180);
        Vector3 beforespecimen2 = new Vector3(46, -53, 180);
        Vector3 rotationbeforespecimen2 = new Vector3(46, -52.5, 90);
        Vector3 atspecimen2 = new Vector3(46, -62, 90);
        Vector3 midwayspecimen2hang = new Vector3(23.5, -47, 0);
        Vector3 specimen2hang = new Vector3(8.75, -32.75, -90);
        Vector3 backmidwayspecimen2spot = new Vector3(23.5, -47, 0);
        Vector3 atspecimen3 = new Vector3(46, -61.5, 90);
        Vector3 midwayspecimen3hang = new Vector3(23.5, -47, 0);
        Vector3 specimen3hang = new Vector3(5.75, -32.75, -90);
        Vector3 parkingposition = new Vector3(54, -54, 0);
        //Vector3 parkingposidtion = new Vector3(54, -54, 0);

        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(humansidestart));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.superSlowSettings));
        autoTasks.addStep(() -> intake.tasks.setMotorsToRunConfig());
        autoTasks.addStep(() -> intake.setHorizontalSlidePosition(-1)); // h-slide in
        // close specimen pincer
        autoTasks.addStep(() -> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(specimenbar, autoTasks);
        autoTasks.addDelay(200);
        autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        positionSolver.addMoveToTaskEx(afterfirstredbar, autoTasks);
        positionSolver.addMoveToTaskEx(rightbeforesample, autoTasks);
        positionSolver.addMoveToTaskEx(atfirstsample, autoTasks);
        positionSolver.addMoveToTaskEx(observationzone1, autoTasks);
        positionSolver.addMoveToTaskEx(beforesecondsample, autoTasks);
        positionSolver.addMoveToTaskEx(atsecondsample, autoTasks);
        positionSolver.addMoveToTaskEx(observationzone2, autoTasks);
        positionSolver.addMoveToTaskEx(beforethirdsample, autoTasks);
        positionSolver.addMoveToTaskEx(atthirdsample, autoTasks);
        positionSolver.addMoveToTaskEx(observationzone3, autoTasks);
        positionSolver.addMoveToTaskEx(beforespecimen2, autoTasks);
        positionSolver.addMoveToTaskEx(rotationbeforespecimen2, autoTasks);
        positionSolver.addMoveToTaskEx(atspecimen2, autoTasks);
        autoTasks.addStep(() -> intake.tasks.startAutoSpecimenPickup()); // grab specimen
        autoTasks.addDelay(1000);
        positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
        autoTasks.addStep(() -> intake.setSpecimenPositions(2)); //
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
        positionSolver.addMoveToTaskEx(specimen2hang, autoTasks);
        autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
        positionSolver.addMoveToTaskEx(backmidwayspecimen2spot, autoTasks);
        positionSolver.addMoveToTaskEx(atspecimen3, autoTasks);
//        autoTasks.addDelay(2000);
//        positionSolver.addMoveToTaskEx(midwayspecimen3hang, autoTasks);
//        positionSolver.addMoveToTaskEx(specimen3hang, autoTasks);
//        autoTasks.addDelay(2000);
//        positionSolver.addMoveToTaskEx(parkingposition, autoTasks);
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
    }

    private void testAuto2(TimedTask autoTasks) {
        Vector3 specimenbar = new Vector3(11.75, -32.75, -90);
        Vector3 afterfirstredbar = new Vector3(36, -40, -90);
        Vector3 specimenpickup = new Vector3(45, -60.5, 90);

        autoTasks.addStep(() -> intake.tasks.startAutoSamplePickup());
        autoTasks.addDelay(5000);
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
//        autoTasks.addStep(() -> intake.setHorizontalSlidePosition(-1));
//        positionSolver.addMoveToTaskEx(specimenbar, autoTasks);
//        positionSolver.addMoveToTaskEx(afterfirstredbar, autoTasks);
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.superSlowSettings));
//        positionSolver.addMoveToTaskEx(specimenpickup, autoTasks);
//        autoTasks.addStep(() -> intake.tasks.startAutoSpecimenPickup());
    }

    private void BucketAuto(TimedTask autoTasks) {
        Vector3 bucketsidestart = new Vector3(-14 - 3.0 / 8.0, -62, -90);
        Vector3 beforespecimenhang = new Vector3(-10, -37.75, -90);
        Vector3 specimenhang = new Vector3(-10, -32.75, -90); //specimen must be lifted before hang
        Vector3 firstsample = new Vector3(-48.8, -38.5, 90);
        Vector3 Highbasketscore = new Vector3(-53.2, -53.7, 43.3);
        Vector3 secondsample = new Vector3(-58.8, -37.39, 90);
        Vector3 Highbasketscore2 = new Vector3(-48.9, -40.9, 40);
        Vector3 thirdsample = new Vector3(-55.3, -24.58, 180);
        Vector3 Highbasketscore3 = new Vector3(-48.9, -40.9, 40);
        Vector3 park = new Vector3(-48.9, -40.9, 40);

        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(bucketsidestart));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.superSlowSettings));
        autoTasks.addStep(() -> intake.tasks.setMotorsToRunConfig());
        autoTasks.addStep(() -> intake.setHorizontalSlidePosition(-1)); // h-slide in
        // close specimen pincer
        autoTasks.addStep(() -> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
        positionSolver.addMoveToTaskEx(beforespecimenhang, autoTasks);
        positionSolver.addMoveToTaskEx(specimenhang, autoTasks);
        autoTasks.addDelay(200);
        autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
        autoTasks.addDelay(200);
        positionSolver.addMoveToTaskEx(beforespecimenhang, autoTasks);
        positionSolver.addMoveToTaskEx(firstsample, autoTasks);
        autoTasks.addDelay(250);
        autoTasks.addStep(() -> intake.tasks.startAutoSamplePickup());
        autoTasks.addDelay(250);
        positionSolver.addMoveToTaskEx(Highbasketscore, autoTasks);
        autoTasks.addDelay(1000);
        autoTasks.addStep(() -> intake.tasks.startAutoBucketLift());
        autoTasks.addStep(() -> intake.tasks.startAutoBucketDropper());
//        autoTasks.addStep(()->
//        positionSolver.addMoveToTaskEx(secondsample, autoTasks);
//        positionSolver.addMoveToTaskEx(Highbasketscore2, autoTasks);
//        positionSolver.addMoveToTaskEx(thirdsample, autoTasks);
//        positionSolver.addMoveToTaskEx(park, autoTasks);
        //positionSolver.addMoveToTaskEx(park, autoTasks);
    }

    private void SpecAuto(TimedTask autoTasks) {
        Vector3 humansidestart = new Vector3(14 + 3.0/8.0, -62, -90);
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

        autoTasks.addStep(()-> intake.stopAllIntakeTasks());
        autoTasks.addStep(()-> odo.setPosition(humansidestart));
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowSettings));
        // close pincer on initial specimen
        autoTasks.addStep(() -> intake.getHardware().specimenServo.setPosition(intake.getSettings().specimenServoClosePosition));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        // raise for specimen hang
        autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
        autoTasks.addDelay(200);
        positionSolver.addMoveToTaskEx(specimenbar, autoTasks);
        autoTasks.addDelay(200);
        // clip specimen on bar
        autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
        autoTasks.addDelay(200);
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        autoTasks.addDelay(200);
        positionSolver.addMoveToTaskEx(afterfirstredbar, autoTasks);
        positionSolver.addMoveToTaskEx(rightbeforesample, autoTasks);
        positionSolver.addMoveToTaskEx(atfirstsample, autoTasks);
        positionSolver.addMoveToTaskEx(observationzone1, autoTasks);
        positionSolver.addMoveToTaskEx(observationzoneprepickup, autoTasks);
        {
            // Second Specimen Hang
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
            positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
            autoTasks.addDelay(200);
            // close pincer on initial specimen
            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenPickup()); // grab specimen
            autoTasks.addDelay(250);
            positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
            autoTasks.addDelay(250);
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
            autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
            autoTasks.addDelay(200);
            positionSolver.addMoveToTaskEx(specimenbar2, autoTasks);
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
        }
        {
            //Moving Third Sample.
            positionSolver.addMoveToTaskEx(afterfirstredbar2, autoTasks);
            positionSolver.addMoveToTaskEx(beforesecondsample, autoTasks);
            positionSolver.addMoveToTaskEx(atsecondsample, autoTasks);
            positionSolver.addMoveToTaskEx(observationzone2, autoTasks);
            positionSolver.addMoveToTaskEx(observationzoneprepickup2, autoTasks);
            autoTasks.addDelay(200);


            // Third Specimen Hang.
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
            positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenPickup()); // grab specimen
            autoTasks.addDelay(250);
            positionSolver.addMoveToTaskEx(midwayspecimen3hang, autoTasks);
            autoTasks.addDelay(250);
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
            autoTasks.addStep(() -> intake.setSpecimenPositions(2)); // prepare for specimen hang
            autoTasks.addDelay(200);
            positionSolver.addMoveToTaskEx(specimenbar3, autoTasks);
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> intake.tasks.startAutoSpecimenHang()); // clip specimen on bar
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
        }
        {
            // Park.
            positionSolver.addMoveToTaskEx(parkingposition, autoTasks);
        }
    }
}