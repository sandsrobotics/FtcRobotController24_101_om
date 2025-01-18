package org.firstinspires.ftc.teamcode;

import static om.self.ezftc.utils.Constants.tileSide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
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

@Config
@Autonomous (name="Flip Auto 2025", group="Test")
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
            if(new EdgeSupplier(()-> robot.opMode.gamepad1.right_bumper).isRisingEdge()) {
                startDelay += 1000;
            }
            else if(new EdgeSupplier(()->robot.opMode.gamepad1.left_bumper).isRisingEdge()) {
                startDelay -= 1000;
                if(startDelay < 0) startDelay = 0;
            } else if(new EdgeSupplier(()->robot.opMode.gamepad1.a).isRisingEdge()) {
                parkPosition = 1;
            } else if(new EdgeSupplier(()->robot.opMode.gamepad1.b).isRisingEdge()) {
                parkPosition = 2;
            } else if(new EdgeSupplier(()->robot.opMode.gamepad1.x).isRisingEdge()) {
                parkPosition = 3;
            } else if(new EdgeSupplier(()->robot.opMode.gamepad1.y).isRisingEdge()) {
                parkPosition = 0;
            }

            if(startDelay > maxDelay) startDelay = maxDelay;

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
        testAuto(autoTasks);

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

    private void testAuto(TimedTask autoTasks) {
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
        Vector3 atfirstsample = new Vector3(44.5, -11.75, 90);
        Vector3 observationzone1 = new Vector3(44.5, -52, 90);
        Vector3 beforesecondsample = new Vector3(44.5, -11.75, 90);
        Vector3 atsecondsample = new Vector3(54.5, -11.75, 90);
        Vector3 observationzone2 = new Vector3(54.5, -52, 90);
        Vector3 observationzoneprepickup = new Vector3(47, -58.5, 90);
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
        //Vector3 parkingposidtion = new Vector3(54, -54, 0);

        autoTasks.addStep(()-> intake.stopAllIntakeTasks());
        autoTasks.addStep(()-> odo.setPosition(humansidestart));
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowSettings));
        // close pincer on initial specimen
        autoTasks.addStep(()-> intake.tasks.getSpecimen.restart());
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        // raise for specimen hang
        autoTasks.addStep(()-> intake.tasks.prepareToHangSpecimenTask.restart());
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(specimenbar, autoTasks);
        autoTasks.addDelay(200);
        // clip specimen on bar
        autoTasks.addStep(()-> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addDelay(200);
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar, autoTasks);
        autoTasks.addDelay(200);
        positionSolver.addMoveToTaskEx(afterfirstredbar, autoTasks);
        positionSolver.addMoveToTaskEx(rightbeforesample, autoTasks);
        positionSolver.addMoveToTaskEx(atfirstsample, autoTasks);
        positionSolver.addMoveToTaskEx(observationzone1, autoTasks);
//        positionSolver.addMoveToTaskEx(beforesecondsample, autoTasks);
//        positionSolver.addMoveToTaskEx(atsecondsample, autoTasks);
//        positionSolver.addMoveToTaskEx(observationzone2, autoTasks);
        positionSolver.addMoveToTaskEx(observationzoneprepickup, autoTasks);
        positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
        //
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowSettings));
        // close pincer on initial specimen
        autoTasks.addStep(()-> intake.tasks.getSpecimen.restart());
        autoTasks.addDelay(250);
        positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
        // raise for specimen hang
        autoTasks.addStep(()-> intake.tasks.prepareToHangSpecimenTask.restart());
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(specimenbar2, autoTasks);
        autoTasks.addDelay(200);
        // clip specimen on bar
        autoTasks.addStep(()-> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addDelay(200);
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar2, autoTasks);
        //After Second Specimen Hang.
        positionSolver.addMoveToTaskEx(afterfirstredbar2, autoTasks);
        positionSolver.addMoveToTaskEx(beforesecondsample, autoTasks);
        positionSolver.addMoveToTaskEx(atsecondsample, autoTasks);
        positionSolver.addMoveToTaskEx(observationzone2, autoTasks);
        positionSolver.addMoveToTaskEx(observationzoneprepickup, autoTasks);
        positionSolver.addMoveToTaskEx(observationzonepickup, autoTasks);
        //
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowSettings));
        // close pincer on initial specimen
        autoTasks.addStep(()-> intake.tasks.getSpecimen.restart());
        autoTasks.addDelay(250);
        positionSolver.addMoveToTaskEx(midwayspecimen3hang, autoTasks);
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);
        // raise for specimen hang
        autoTasks.addStep(()-> intake.tasks.prepareToHangSpecimenTask.restart());
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(specimenbar3, autoTasks);
        autoTasks.addDelay(200);
        // clip specimen on bar
        autoTasks.addStep(()-> intake.tasks.hangSpecimenTask.restart());
        autoTasks.addDelay(200);
        autoTasks.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(rightbeforespecimenbar3, autoTasks);

//

//        autoTasks.addStep(()-> intake.tasks.getSpecimen.restart());
//        autoTasks.addDelay(250);
//        autoTasks.addStep(()-> intake.tasks.prepareToHangSpecimenTask.restart());
//        autoTasks.addDelay(500);
//        positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
//        positionSolver.addMoveToTaskEx(specimen2hang, autoTasks);
//        autoTasks.addDelay(500);
//        autoTasks.addStep(()-> intake.tasks.hangSpecimenTask.restart());

//        positionSolver.addMoveToTaskEx(beforethirdsample, autoTasks);
//        positionSolver.addMoveToTaskEx(atthirdsample, autoTasks);
//        positionSolver.addMoveToTaskEx(observationzone3, autoTasks);
//        positionSolver.addMoveToTaskEx(beforespecimen2, autoTasks);
//        positionSolver.addMoveToTaskEx(rotationbeforespecimen2, autoTasks);
//        positionSolver.addMoveToTaskEx(atspecimen2, autoTasks);
//        // grab specimen
//        positionSolver.addMoveToTaskEx(midwayspecimen2hang, autoTasks);
//        // raise for specimen hang
//        positionSolver.addMoveToTaskEx(specimen2hang, autoTasks);
//        // clip specimen on bar
//        positionSolver.addMoveToTaskEx(backmidwayspecimen2spot, autoTasks);
//        positionSolver.addMoveToTaskEx(atspecimen3, autoTasks);
//        positionSolver.addMoveToTaskEx(midwayspecimen3hang, autoTasks);
//        positionSolver.addMoveToTaskEx(specimen3hang, autoTasks);
//        positionSolver.addMoveToTaskEx(parkingposition, autoTasks);
//        autoTasks.addStep(()-> intake.stopAllIntakeTasks());
    }

    private void testBucketAuto(TimedTask autoTasks) {
        Vector3 bucketstartposition = new Vector3(-14 - 3.0/8.0, -60.5, 90);
        Vector3 specimenpickup = new Vector3(45, -60.5, 90);
        
    }
}